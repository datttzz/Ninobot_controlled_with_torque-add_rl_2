#!/usr/bin/env python3
"""Tuning riêng PID có ngân sách, protocol cố định và xác minh tách biệt."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
from time import monotonic, sleep, time

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from benchmark_pi_pid_sac import episode_metrics, filtered_jerk, read_rows, rms, stats
from rl_training.ablation_suite import active_gazebo, save_json, sha
from rl_training.experiment_guard import active_jobs, ResourceGuard, ResourceStop
from rl_training.mission_measurement import mission_conditions
from rl_training.frame_registration import load_registration,registration_from_metadata

# Nhánh gain thấp hơn/nhẹ D hơn dựa trên tuning cũ: tăng Kp/Kd đã tăng limiter.
# Tập cố định trước khi chạy; không tạo ứng viên mới sau khi xem kết quả.
CANDIDATES=[
 dict(name='p025_i010_d0002_t050',velocity_kp=.25,velocity_ki=.10,velocity_kd=.0002,derivative_filter_seconds=.05),
 dict(name='p030_i010_d0002_t080',velocity_kp=.30,velocity_ki=.10,velocity_kd=.0002,derivative_filter_seconds=.08),
 dict(name='p030_i010_d0005_t050',velocity_kp=.30,velocity_ki=.10,velocity_kd=.0005,derivative_filter_seconds=.05),
 dict(name='p035_i010_d0002_t080',velocity_kp=.35,velocity_ki=.10,velocity_kd=.0002,derivative_filter_seconds=.08),
 dict(name='p025_i012_d0005_t080',velocity_kp=.25,velocity_ki=.12,velocity_kd=.0005,derivative_filter_seconds=.08),
 dict(name='p040_i010_d0005_t080',velocity_kp=.40,velocity_ki=.10,velocity_kd=.0005,derivative_filter_seconds=.08),
]


def measurement(folder,registration=None):
    folder=Path(folder);ref=json.loads((folder.parent/'reference_path.json').read_text())
    metadata=json.loads((folder/'metadata.json').read_text())
    registration=registration_from_metadata(dict(registration=registration) if registration else metadata,metadata['map'],required=True)
    episodes=[]
    for session in sorted(folder.glob('episode_*')):
        raw=session/'telemetry.jsonl'
        if not raw.exists():continue
        rows=read_rows(raw)
        if not rows or rows[-1].get('outcome') is None:continue
        result=episode_metrics(rows,ref,map_to_world=registration['map_to_world']);m=result['metrics']
        if m['mission_success_pct'] is None:
            raise ValueError('GT tuning blocked: missing source/terminal coverage; no extrapolation')
        status=mission_conditions(result['outcome'],m['amcl_goal_xy_m'],m['amcl_goal_yaw_rad'],
                   m['amcl_max_m'],m['gt_goal_xy_m'],m['gt_goal_yaw_rad'],m['gt_max_m'])
        t=np.array([r['simulation_time'] for r in rows]);weights=np.diff(t,prepend=0)
        roll=np.array([r.get('roll',float('nan')) for r in rows])
        pitch=np.array([r.get('pitch',float('nan')) for r in rows])
        gt_time=np.array([r['gt_stamp_s'] for r in rows])
        # Dùng đúng timestamp GT, lọc đạo hàm e_y ở 5Hz; đơn vị m/s, không gọi là jerk.
        oscillation=filtered_jerk(result['trace']['error'],gt_time)
        wheel=np.array([r['wheel_velocity'] for r in rows]);target=np.array([r['wheel_target'] for r in rows])
        m.update(lateral_rate_rms_m_s=rms(oscillation,weights),roll_angle_rms_rad=rms(roll,weights),
                 roll_angle_peak_rad=float(np.max(np.abs(roll))),pitch_angle_peak_rad=float(np.max(np.abs(pitch))),
                 measured_wheel_peak_rad_s=float(np.max(np.abs(wheel))),target_wheel_peak_rad_s=float(np.max(np.abs(target))))
        episodes.append(dict(session=str(session),**status,outcome=result['outcome'],metrics=m,
                      failure_reasons=result['mission_failure_reasons'],coverage=result['coverage'],
                      raw_sha256=sha(raw)))
    if not episodes:raise ValueError('Không có episode terminal hoàn chỉnh: '+str(folder))
    return dict(episodes=episodes,metrics={k:stats(e['metrics'].get(k) for e in episodes) for k in episodes[0]['metrics']})


def rejection_reasons(value):
    reasons=[]
    for i,e in enumerate(value['episodes'],1):
        m=e['metrics']
        if e['full_pass'] is not True:reasons.append(f'episode {i}: full task failed: '+', '.join(e['failure_reasons']))
        for key,limit in [('roll_angle_peak_rad',.5),('pitch_angle_peak_rad',.5),
                          ('measured_wheel_peak_rad_s',24.),('target_wheel_peak_rad_s',12.),('torque_peak_nm',2.)]:
            v=m.get(key)
            if v is None or not math.isfinite(v) or v>limit+1e-9:reasons.append(f'episode {i}: {key} missing/exceeds {limit}')
        if e['coverage']['ground_truth']<.99 or e['coverage']['az']<.99:reasons.append(f'episode {i}: missing measurement coverage')
    return reasons


def ranking(value):
    # Lexicographic, không điểm tổng hợp hoặc trọng số tùy tiện.
    return tuple(value['metrics'][k]['mean'] if value['metrics'][k]['mean'] is not None else math.inf for k in
                 ('wheel_rmse_rad_s','gt_rmse_m','lateral_rate_rms_m_s','roll_angle_rms_rad'))


def combine(*values):
    episodes=[e for v in values for e in v['episodes']]
    return dict(episodes=episodes,metrics={k:stats(e['metrics'].get(k) for e in episodes) for k in episodes[0]['metrics']})


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--budget-minutes',type=float,default=60.)
    parser.add_argument('--registration',help='Nguồn map→world độc lập bắt buộc cho GT tuning')
    args=parser.parse_args(argv)
    if not 0<args.budget_minutes<=60:parser.error('Tối đa 60 phút')
    registration=load_registration(args.registration,ROOT/'maps/hall_01.yaml',required=True)
    if active_jobs() or active_gazebo():parser.error('Đang có job/Gazebo khác; không chạy trùng')
    out=args.output_root.resolve();out.mkdir(parents=True,exist_ok=False)
    start=time();deadline=monotonic()+args.budget_minutes*60-45
    guard=ResourceGuard(out,dict(deadline_monotonic=deadline))
    protocol=dict(created_utc=datetime.now(timezone.utc).isoformat(),start_unix=start,
       budget_minutes=args.budget_minutes,deadline_monotonic=deadline,candidates=CANDIDATES,
       registration=registration,
       groups=['PID'],residual_enabled=False,screening=dict(episodes=1,seed=5101,max_seconds=90),
       selection=dict(max_finalists=2,additional_episodes=2,seed=5201),verification=dict(episodes=3,seed=6101),
       hard_criteria=dict(all_full_task_pass=True,xy_m=.25,yaw_rad=.25,max_deviation_m=.20,
          roll_pitch_peak_rad=.5,measured_wheel_peak_rad_s=24.,target_wheel_peak_rad_s=12.,torque_peak_nm=2.),
       ranking='lexicographic: wheel RMSE, GT lateral RMSE, filtered lateral rate RMS, roll angle RMS',
       limiter_policy='Report slew/saturation percentages; no zero-slew requirement, no limit changes',
       fallback='If no eligible candidate: freeze one provisional profile for 3 diagnostic verification episodes; never deploy it',
       fallback_ranking='most full-task passes, fewest unsafe episodes, then same lexicographic metrics; not an eligible PID',
       prior_tuning='rl_training/runs/pid_tuning_20261030_measured_D; same plant/limits, old full heldouts fail new hard constraints; no rerun of those gains',
       prior_PI='Reuse existing PI data only as context, no new PI/SAC run and no superiority assertion',
       resources=dict(min_ram_gib=2,min_disk_gib=3,temperature_c=95,consecutive_hot_checks=3),
       source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__).resolve(),ROOT/'rl_training/ros_env.py',
                     ROOT/'rl_training/mission_measurement.py',ROOT/'src/nino_control/nino_control/effort_drive.py',
                     ROOT/'src/nino_control/nino_control/derivative.py',ROOT/'src/nino_navigation/config/nav2.yaml',
                     ROOT/'src/nino_control/config/effort_drive.yaml']})
    save_json(out/'protocol.json',protocol)
    (out/'operating_pid_before.yaml').write_bytes((ROOT/'rl_training/config/pid.yaml').read_bytes())
    state=dict(state='RUNNING',trials={},selected=None,profile_updated=False)
    def persist():save_json(out/'state.json',state)
    persist()
    def trial(name,gain,episodes,seed,purpose='calibration'):
        guard.check(force=True)
        if deadline-monotonic()<180:raise ResourceStop('Không đủ ngân sách cho trial mới và cleanup')
        profile=out/(name+'_gains.yaml')
        profile.write_text(yaml.safe_dump({k:v for k,v in gain.items() if k!='name'}))
        folder=out/name
        cmd=[sys.executable,str(ROOT/'run_ablation_suite.py'),'--modes','PID','--pid-config',str(profile),
             '--episodes',str(episodes),'--seed',str(seed),'--purpose',purpose,'--device','cpu',
             '--max-episode-seconds','90','--deadline-monotonic',str(deadline),'--output-root',str(folder)]
        cmd+=['--registration',registration['path']]
        state['current_trial']=name;state['trials'][name]=dict(state='RUNNING',command=cmd);persist()
        print('PID only:',name,flush=True)
        with (out/(name+'.log')).open('w') as log:
            child=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while child.poll() is None:
                    snapshot=guard.check()
                    with (out/'resources.jsonl').open('a') as f:f.write(json.dumps(dict(unix=time(),trial=name,**snapshot))+'\n')
                    sleep(2)
            except BaseException:
                child.send_signal(signal.SIGINT);child.wait(timeout=50);raise
        if child.returncode or active_gazebo():
            state['trials'][name].update(state='INFRASTRUCTURE_ERROR',exit_code=child.returncode);persist()
            raise RuntimeError('Lỗi hạ tầng: '+str(out/(name+'.log')))
        measured=measurement(folder/'PID',registration);reasons=rejection_reasons(measured)
        state['trials'][name].update(state='COMPLETE',measurement=measured,rejection_reasons=reasons);persist()
        return measured
    try:
        measured={};eligible=[]
        for c in CANDIDATES:
            value=trial('screen_'+c['name'],c,1,5101);measured[c['name']]=value
            if not rejection_reasons(value):eligible.append(c)
        finalists=sorted(eligible,key=lambda c:ranking(measured[c['name']]))[:2]
        state['finalists']=[c['name'] for c in finalists];persist()
        qualified=[]
        for c in finalists:
            value=trial('selection_'+c['name'],c,2,5201)
            measured[c['name']]=combine(measured[c['name']],value)
            if not rejection_reasons(measured[c['name']]):qualified.append(c)
        approved=bool(qualified)
        if approved:chosen=min(qualified,key=lambda c:ranking(measured[c['name']]))
        else:
            def fallback(c):
                value=measured[c['name']]
                passed=sum(e['full_pass'] is True for e in value['episodes'])
                return (-passed,len(rejection_reasons(value)),*ranking(value))
            chosen=min(CANDIDATES,key=fallback)
        state.update(selected=chosen,selection_eligible=approved,selection_measurement=measured[chosen['name']],
                     freeze_unix=time(),verification_used_for_tuning=False)
        frozen=out/'pid_frozen_candidate.yaml'
        frozen.write_text('# Gain khóa trước 3 lượt xác minh; chỉ được deploy nếu state.profile_updated=true.\n'+
                          yaml.safe_dump({k:v for k,v in chosen.items() if k!='name'}))
        state['frozen_profile_sha256']=sha(frozen);persist()
        verification=trial('verification',chosen,3,6101,'evaluation')
        reasons=rejection_reasons(verification)
        state.update(state='COMPLETE',verification=verification,verification_rejections=reasons,
                     profile_updated=approved and not reasons,elapsed_seconds=time()-start)
        if state['profile_updated']:
            operating=ROOT/'rl_training/config/pid.yaml'
            operating.write_text('# PID xác minh trên hall_01; xem PID_CALIBRATION.json. Không tối ưu toàn cục.\n'+
                                 yaml.safe_dump({k:v for k,v in chosen.items() if k!='name'}))
            old_metadata=ROOT/'rl_training/config/PID_CALIBRATION.json'
            if old_metadata.exists():(out/'previous_PID_CALIBRATION.json').write_bytes(old_metadata.read_bytes())
            save_json(old_metadata,dict(tuning_dir=str(out),**state))
        state['operating_profile_sha256']=sha(ROOT/'rl_training/config/pid.yaml');persist()
        print(json.dumps(dict(selected=chosen,selection_eligible=approved,verification_rejections=reasons,
                              profile_updated=state['profile_updated'])),flush=True)
    except BaseException as exc:
        state.update(state='INTERRUPTED' if isinstance(exc,KeyboardInterrupt) else 'STOPPED',
                     reason=str(exc),elapsed_seconds=time()-start,profile_updated=False);persist()
        print(json.dumps(dict(state=state['state'],reason=str(exc))),flush=True)
        return 130 if isinstance(exc,KeyboardInterrupt) else 2
    return 0


if __name__=='__main__':raise SystemExit(main())
