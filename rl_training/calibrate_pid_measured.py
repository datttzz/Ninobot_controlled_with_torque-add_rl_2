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


from rl_training.pid_tuning_data import measurement,rejection_reasons,ranking,combine,compatibility,GAIN_KEYS


def main(argv=None):
    parser=argparse.ArgumentParser(description='Tái phân tích log PID cũ, khóa gain và chỉ xác minh 2 episode mới')
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--budget-minutes',type=float,default=30.)
    args=parser.parse_args(argv)
    if not 0<args.budget_minutes<=30:parser.error('Tối đa 30 phút')
    if active_jobs() or active_gazebo():parser.error('Đang có job/Gazebo khác; không chạy trùng')
    out=args.output_root.resolve();out.mkdir(parents=True,exist_ok=False)
    start=time();deadline=monotonic()+args.budget_minutes*60-45
    guard=ResourceGuard(out,dict(deadline_monotonic=deadline))
    original=(ROOT/'rl_training/config/pid.yaml').read_bytes()
    (out/'operating_pid_before.yaml').write_bytes(original)
    operating=yaml.safe_load(original)
    candidates=CANDIDATES+([dict(name='operating',**operating)] if not any(all(c[k]==operating[k] for k in GAIN_KEYS) for c in CANDIDATES) else [])
    roots=[ROOT/'rl_training/runs'/name for name in ('pid_measured_tuning_20261030','pid_tuning_20261030_measured_D',
           'measurement_feedback_fixes_20261031/integration_PID','amcl_only_verification_20261031/run')]
    protocol=dict(created_utc=datetime.now(timezone.utc).isoformat(),start_unix=start,
       budget_minutes=args.budget_minutes,deadline_monotonic=deadline,candidates=candidates,
       measurement_basis='AMCL_IMU_ENCODER_v1',groups=['PID'],residual_enabled=False,
       new_episode_budget=2,new_screening_episodes=0,verification=dict(episodes=2,seed=9301),
       user_override='Latest request: two new simulation episodes total, replaces earlier three-episode requirement',
       reuse_roots=[str(p) for p in roots],
       hard_criteria=dict(nav2_succeeded=True,amcl_xy_m=.25,amcl_yaw_rad=.25,amcl_max_deviation_m=.20,
            roll_pitch_peak_rad=.5,measured_wheel_peak_rad_s=24.,target_wheel_peak_rad_s=12.,torque_peak_nm=2.,
            source_coverage_min=.99,no_invalid_feedback=True,no_nonfinite_signals=True),
       ranking='lexicographic: wheel-speed RMSE, AMCL lateral RMSE, gravity-compensated body IMU ay RMS',
       fallback='If no historical candidate satisfies all checks: provisional candidate with most independently verified AMCL passes, then same ranking; two held-out passes required to select',
       historical_policy='Recompute raw AMCL/IMU/encoder; old acceptance labels are ignored. Missing terminal brackets remain unknown; known AMCL failures are retained. Historical evidence is not a new validation episode.',
       limits='Unchanged: no slew=0 requirement, no reference-speed reduction, no PI or SAC modification',
       source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__).resolve(),ROOT/'rl_training/pid_tuning_data.py',ROOT/'benchmark_pi_pid_sac.py',ROOT/'rl_training/ros_env.py',ROOT/'src/nino_control/nino_control/effort_drive.py',ROOT/'src/nino_control/nino_control/derivative.py',ROOT/'src/nino_navigation/config/nav2.yaml']})
    save_json(out/'protocol.json',protocol)  # Khóa trước khi đọc metric ứng viên.
    state=dict(state='REPROCESSING',profile_updated=False,verification_used_for_tuning=False,reuse=[],selected=None)
    def persist():save_json(out/'state.json',state)
    persist()
    def identity(gain):return tuple(gain[k] for k in GAIN_KEYS)
    aggregate={}
    try:
        for root in roots:
            for path in sorted(root.rglob('metadata.json')):
                guard.check(force=True)
                meta=json.loads(path.read_text())
                if meta.get('controller_mode')!='PID':continue
                record=dict(folder=str(path.parent))
                try:
                    gain=meta['gains'];key=identity(gain)
                    if key not in [identity(c) for c in candidates]:
                        record.update(status='outside_predeclared_set');state['reuse'].append(record);continue
                    check=compatibility(path.parent);record['compatibility']=check
                    if not check['compatible']:
                        record.update(status='incompatible',reasons=check['reasons']);state['reuse'].append(record);continue
                    value=measurement(path.parent)
                    record.update(status='reprocessed',gains=gain,measurement=value,rejection_reasons=rejection_reasons(value))
                    aggregate[key]=combine(aggregate[key],value) if key in aggregate else value
                except (ValueError,KeyError,FileNotFoundError) as exc:
                    record.update(status='insufficient_data',reason=str(exc))
                state['reuse'].append(record);persist()
        eligible=[c for c in candidates if identity(c) in aggregate and not rejection_reasons(aggregate[identity(c)] )]
        if eligible:
            chosen=min(eligible,key=lambda c:ranking(aggregate[identity(c)]));selection_eligible=True
        else:
            available=[c for c in candidates if identity(c) in aggregate]
            if not available:raise ValueError('No compatible raw PID data; two-episode budget cannot support an additional screening campaign')
            def fallback(c):
                value=aggregate[identity(c)]
                return (-sum(e['mission_pass'] is True for e in value['episodes']),*ranking(value))
            chosen=min(available,key=fallback);selection_eligible=False
        frozen=out/'pid_frozen_candidate.yaml'
        frozen.write_text('# Khóa trước đúng 2 lượt xác minh; không chỉnh theo kết quả xác minh.\n'+yaml.safe_dump({k:chosen[k] for k in GAIN_KEYS}))
        frozen_hash=sha(frozen)
        state.update(state='FROZEN',selected=chosen,selection_eligible=selection_eligible,
                     selection_measurement=aggregate[identity(chosen)],freeze_unix=time(),frozen_profile_sha256=frozen_hash)
        persist()
        guard.check(force=True)
        if deadline-monotonic()<240:raise ResourceStop('Không đủ ngân sách cho hai lượt xác minh và cleanup')
        cmd=[sys.executable,str(ROOT/'run_ablation_suite.py'),'--modes','PID','--pid-config',str(frozen),
             '--episodes','2','--seed','9301','--purpose','evaluation','--device','cpu',
             '--deadline-monotonic',str(deadline),'--output-root',str(out/'verification')]
        state.update(state='VERIFYING',verification_command=cmd);persist()
        with (out/'verification.log').open('w') as log:
            child=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while child.poll() is None:
                    snapshot=guard.check()
                    with (out/'resources.jsonl').open('a') as f:f.write(json.dumps(dict(unix=time(),**snapshot))+'\n')
                    sleep(2)
            except BaseException:
                child.send_signal(signal.SIGINT);child.wait(timeout=50);raise
        if child.returncode or active_gazebo():raise RuntimeError('Verification infrastructure fault; logs retained, no additional run')
        assert sha(frozen)==frozen_hash
        verified=measurement(out/'verification/PID')
        reasons=rejection_reasons(verified,require_feedback=True)
        if len(verified['episodes'])!=2:reasons.append('Expected exactly two terminal episodes')
        state.update(state='COMPLETE',verification=verified,verification_rejections=reasons,
                     profile_updated=not reasons,elapsed_seconds=time()-start)
        if not reasons:
            operating_path=ROOT/'rl_training/config/pid.yaml'
            operating_path.write_text('# PID xác minh AMCL/IMU/encoder trên hall_01 qua 2 lượt; không tối ưu toàn cục.\n'+yaml.safe_dump({k:chosen[k] for k in GAIN_KEYS}))
            record_path=ROOT/'rl_training/config/PID_CALIBRATION.json'
            if record_path.exists():(out/'previous_PID_CALIBRATION.json').write_bytes(record_path.read_bytes())
            save_json(record_path,dict(measurement_basis='AMCL_IMU_ENCODER_v1',tuning_dir=str(out),**state))
        else:
            assert (ROOT/'rl_training/config/pid.yaml').read_bytes()==original
        state['operating_profile_sha256']=sha(ROOT/'rl_training/config/pid.yaml');persist()
        print(json.dumps(dict(selected=chosen,verification_rejections=reasons,profile_updated=state['profile_updated'],episodes=len(verified['episodes']))),flush=True)
    except BaseException as exc:
        state.update(state='INTERRUPTED' if isinstance(exc,KeyboardInterrupt) else 'STOPPED',reason=str(exc),elapsed_seconds=time()-start,profile_updated=False);persist()
        print(json.dumps(dict(state=state['state'],reason=str(exc))),flush=True)
        return 130 if isinstance(exc,KeyboardInterrupt) else 2
    return 0


if __name__=='__main__':raise SystemExit(main())
