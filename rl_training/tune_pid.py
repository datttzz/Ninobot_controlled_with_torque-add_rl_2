#!/usr/bin/env python3
"""Hiệu chỉnh hữu hạn có protocol chốt trước; không train hay sửa SAC/PI/giới hạn."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from time import monotonic,sleep,time

import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rl_training.ablation_suite import active_gazebo,save_json,sha
from rl_training.experiment_guard import active_jobs,ResourceGuard,ResourceStop
from benchmark_pi_pid_sac import episode_metrics,read_rows,stats
from rl_training.frame_registration import load_registration,registration_from_metadata

CANDIDATES=[
 dict(name='p035_d0005_t050',velocity_kp=.35,velocity_ki=.10,velocity_kd=.0005,derivative_filter_seconds=.05),
 dict(name='p040_d0010_t080',velocity_kp=.40,velocity_ki=.10,velocity_kd=.0010,derivative_filter_seconds=.08),
 dict(name='p045_d0020_t050',velocity_kp=.45,velocity_ki=.12,velocity_kd=.0020,derivative_filter_seconds=.05),
 dict(name='p055_d0030_t080',velocity_kp=.55,velocity_ki=.15,velocity_kd=.0030,derivative_filter_seconds=.08),
 dict(name='p060_d0010_t080',velocity_kp=.60,velocity_ki=.15,velocity_kd=.0010,derivative_filter_seconds=.08),
 dict(name='p040_d0005_t030',velocity_kp=.40,velocity_ki=.12,velocity_kd=.0005,derivative_filter_seconds=.03),
]
CRITERIA=dict(wheel_rmse_min_improvement_fraction=.05,max_gt_rmse_increase_m=.005,
              max_amcl_rmse_increase_m=.005,max_slew_increase_percentage_points=1.0,
              max_saturation_increase_percentage_points=.1,
              mission_success_noninferior=True,nav_success_noninferior=True,
              max_torque_nm=2.0,limits_unchanged=True,
              selection='Eligible held-out candidates: lowest wheel RMSE, then GT RMSE, then slew fraction')


def measure(folder,label,registration=None):
    folder=Path(folder)/label
    metadata=json.loads((folder/'metadata.json').read_text())
    registration=registration_from_metadata(dict(registration=registration) if registration else metadata,metadata['map'],required=True)
    reference=json.loads((folder.parent/'reference_path.json').read_text())
    episodes=[]
    for session in sorted(folder.glob('episode_*')):
        path=session/'telemetry.jsonl'
        if not path.exists():continue
        metrics=episode_metrics(read_rows(path),reference,map_to_world=registration['map_to_world'])
        if metrics['outcome']=='INCOMPLETE':continue
        if metrics['coverage']['ground_truth']<.99 or metrics['coverage']['az']<.99:
            raise ValueError('Insufficient measured GT/IMU coverage')
        if metrics['metrics']['mission_success_pct'] is None:
            raise ValueError('GT tuning blocked: missing synchronized terminal measurement')
        episodes.append(metrics)
    if not episodes:raise ValueError('No complete calibration episodes')
    return dict(episodes=len(episodes),outcomes=[e['outcome'] for e in episodes],
                duration_min_s=min(e['duration_s'] for e in episodes),
                metrics={key:stats(e['metrics'][key] for e in episodes) for key in episodes[0]['metrics']})


def eligibility(candidate,baseline):
    c,b=candidate['metrics'],baseline['metrics'];reasons=[]
    def mean(key):
        if c[key]['mean'] is None or b[key]['mean'] is None:reasons.append('Missing '+key);return None,None
        return c[key]['mean'],b[key]['mean']
    for key in ('mission_success_pct','nav_success_pct'):
        x,y=mean(key)
        if x is not None and x<y:reasons.append(key+' inferior')
    for key,increase in [('gt_rmse_m',.005),('amcl_rmse_m',.005),('slew_limit_pct',1.),('saturation_pct',.1)]:
        x,y=mean(key)
        if x is not None and x>y+increase:reasons.append(key+' exceeds predefined margin')
    x,y=mean('wheel_rmse_rad_s')
    if x is not None and not x<=.95*y:reasons.append('Wheel RMSE improvement below 5%')
    peak=c['torque_peak_nm']['p95']
    if peak is None or peak>2.0+1e-9:reasons.append('Torque operating limit violation/missing')
    if candidate['episodes']!=baseline['episodes']:reasons.append('Unequal held-out episode counts')
    return reasons


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',required=True,type=Path)
    parser.add_argument('--budget-minutes',type=float,default=60.)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--registration',help='Nguồn map→world độc lập bắt buộc cho GT tuning')
    args=parser.parse_args(argv)
    if not 0<args.budget_minutes<=60:parser.error('Budget must be >0 and <=60 minutes')
    registration=load_registration(args.registration,ROOT/'maps/hall_01.yaml',required=True)
    jobs=active_jobs()
    if jobs or active_gazebo():parser.error(f'Jobs/Gazebo active: {jobs}, {active_gazebo()}')
    output=args.output_root.resolve()
    if args.resume:
        protocol=json.loads((output/'protocol.json').read_text())
        if protocol.get('registration')!=registration:parser.error('Registration differs/missing in original tuning protocol')
    else:
        output.mkdir(parents=True,exist_ok=False)
        protocol=dict(created_utc=datetime.now(timezone.utc).isoformat(),start_unix=time(),
           budget_seconds=args.budget_minutes*60,deadline_monotonic=monotonic()+args.budget_minutes*60-60,
           registration=registration,
           boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
           candidates=CANDIDATES,criteria=CRITERIA,
           screening=dict(episodes=1,seed=201,max_episode_seconds=30),
           held_out=dict(episodes=2,seed=301,max_episode_seconds=90,finalists=2),
           ordering='PI screening, all PID candidates, PI held-out, two finalists held-out',
           sources={str(ROOT/p):sha(ROOT/p) for p in ('rl_training/tune_pid.py','rl_training/ablation_suite.py',
             'rl_training/ros_env.py','benchmark_pi_pid_sac.py','src/nino_control/nino_control/effort_drive.py','src/nino_control/nino_control/derivative.py')},
           resources=dict(min_available_ram_gib=2,min_free_disk_gib=3,max_temperature_c=95,hot_consecutive_checks=3),
           caveats=['Gym seeds do not control Gazebo/AMCL noise','Screening TIME_LIMIT is intentional at 30s',
                    'All tuning logs retained; independent evaluation uses different seeds and new run',
                    'No global optimality or superiority is assumed; profile updated only if criteria pass'])
        save_json(output/'protocol.json',protocol)
        (output/'original_pid.yaml').write_text((ROOT/'rl_training/config/pid.yaml').read_text())
    for path,digest in protocol['sources'].items():
        if sha(path)!=digest:parser.error('Source changed since tuning protocol: '+path)
    if protocol['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        parser.error('Machine rebooted; do not extend this wall-clock tuning budget. Keep logs and start a new protocol.')
    deadline=protocol['deadline_monotonic']
    guard=ResourceGuard(output,dict(deadline_monotonic=deadline))
    state=json.loads((output/'state.json').read_text()) if (output/'state.json').exists() else dict(trials={},state='RUNNING')
    def persist():save_json(output/'state.json',state)
    def trial(name,mode,episodes,seed,seconds,gains=None):
        if name in state['trials'] and state['trials'][name]['state']=='complete':return state['trials'][name]['measurement']
        guard.check(force=True)
        if deadline-monotonic()<180:raise ResourceStop('Not enough remaining budget for a new trial and cleanup')
        directory=output/name
        gain_path=output/(name+'_gains.yaml')
        if gains:gain_path.write_text(yaml.safe_dump({k:v for k,v in gains.items() if k!='name'}))
        command=[sys.executable,str(ROOT/'run_ablation_suite.py'),'--modes',mode,
          '--episodes',str(episodes),'--seed',str(seed),'--max-episode-seconds',str(seconds),
          '--purpose','calibration','--device','cpu','--deadline-monotonic',str(deadline),
          '--output-root',str(directory)]
        command+=['--registration',registration['path']]
        if gains:command+=['--pid-config',str(gain_path)]
        if directory.exists():command+=['--resume']
        state['current_trial']=name;state['trials'][name]=dict(state='running',command=command);persist()
        print('Calibration:',name,flush=True)
        with (output/(name+'.log')).open('a') as log:
            child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while child.poll() is None:
                    snapshot=guard.check()
                    with (output/'resources.jsonl').open('a') as resource:
                        resource.write(json.dumps(dict(unix=time(),trial=name,**snapshot))+'\n')
                    sleep(2)
                code=child.returncode
            except (KeyboardInterrupt,ResourceStop):
                child.send_signal(signal.SIGINT)
                child.wait(timeout=50)
                raise
        if code or active_gazebo():
            state['trials'][name].update(state='infrastructure_error',exit_code=code);persist()
            raise RuntimeError('Trial infrastructure error; inspect '+str(output/(name+'.log')))
        value=measure(directory,mode,registration)
        state['trials'][name].update(state='complete',measurement=value);persist()
        return value
    try:
        trial('screen_PI','PI',1,201,30)
        screening=[]
        for candidate in protocol['candidates']:
            value=trial('screen_'+candidate['name'],'PID',1,201,30,candidate)
            if value['duration_min_s']>=29 or value['outcomes']==['SUCCEEDED']:
                if 'FAILED' not in value['outcomes']:
                    screening.append((value['metrics']['wheel_rmse_rad_s']['mean'],candidate))
        shortlist=[c for _,c in sorted(screening,key=lambda pair:pair[0])[:2]]
        state['shortlist']=[c['name'] for c in shortlist];persist()
        baseline=trial('heldout_PI','PI',2,301,90)
        eligible=[];assessment=[]
        for candidate in shortlist:
            value=trial('heldout_'+candidate['name'],'PID',2,301,90,candidate)
            reasons=eligibility(value,baseline)
            assessment.append(dict(candidate=candidate,measurement=value,rejection_reasons=reasons))
            if not reasons:
                eligible.append((value['metrics']['wheel_rmse_rad_s']['mean'],value['metrics']['gt_rmse_m']['mean'],
                                 value['metrics']['slew_limit_pct']['mean'],candidate))
        selected=min(eligible,key=lambda item:item[:3])[-1] if eligible else None
        state.update(state='COMPLETE',selected=selected,assessment=assessment,baseline=baseline,
                     profile_updated=selected is not None,elapsed_seconds=time()-protocol['start_unix'])
        if selected:
            gain={k:v for k,v in selected.items() if k!='name'}
            path=ROOT/'rl_training/config/pid.yaml'
            path.write_text('# Selected in bounded tuning; see PID_CALIBRATION.json. Not globally optimal.\n'+yaml.safe_dump(gain))
        state['profile_sha256']=sha(ROOT/'rl_training/config/pid.yaml');persist()
        save_json(ROOT/'rl_training/config/PID_CALIBRATION.json',dict(tuning_dir=str(output),**state))
        print('Selected:',selected if selected else 'none; no profile update',flush=True)
    except (KeyboardInterrupt,ResourceStop,Exception) as exc:
        state.update(state='INTERRUPTED' if isinstance(exc,KeyboardInterrupt) else 'STOPPED',reason=str(exc),
                     elapsed_seconds=time()-protocol['start_unix'],profile_updated=False)
        persist()
        print(json.dumps(dict(state=state['state'],reason=str(exc))),flush=True)
        return 130 if isinstance(exc,KeyboardInterrupt) else 2
    return 0


if __name__=='__main__':raise SystemExit(main())
