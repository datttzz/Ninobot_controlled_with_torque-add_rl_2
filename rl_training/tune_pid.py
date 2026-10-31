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

CANDIDATES=[
 dict(name='p035_d0005_t050',velocity_kp=.35,velocity_ki=.10,velocity_kd=.0005,derivative_filter_seconds=.05),
 dict(name='p040_d0010_t080',velocity_kp=.40,velocity_ki=.10,velocity_kd=.0010,derivative_filter_seconds=.08),
 dict(name='p045_d0020_t050',velocity_kp=.45,velocity_ki=.12,velocity_kd=.0020,derivative_filter_seconds=.05),
 dict(name='p055_d0030_t080',velocity_kp=.55,velocity_ki=.15,velocity_kd=.0030,derivative_filter_seconds=.08),
 dict(name='p060_d0010_t080',velocity_kp=.60,velocity_ki=.15,velocity_kd=.0010,derivative_filter_seconds=.08),
 dict(name='p040_d0005_t030',velocity_kp=.40,velocity_ki=.12,velocity_kd=.0005,derivative_filter_seconds=.03),
]
CRITERIA=dict(wheel_rmse_min_improvement_fraction=.05,max_amcl_rmse_increase_m=.005,max_slew_increase_percentage_points=1.0,
              max_saturation_increase_percentage_points=.1,
              mission_success_noninferior=True,nav_success_noninferior=True,
              max_torque_nm=2.0,limits_unchanged=True,
              selection='wheel RMSE, then AMCL RMSE, then gravity-compensated lateral IMU RMS')


def measure(folder,label):
    from rl_training.pid_tuning_data import measurement,compatibility
    folder=Path(folder)/label
    check=compatibility(folder,expected_mode=label)
    if not check['compatible']:raise ValueError('; '.join(check['reasons']))
    value=measurement(folder)
    return dict(episodes=len(value['episodes']),outcomes=[e['outcome'] for e in value['episodes']],
                duration_min_s=min(e['duration_s'] for e in value['episodes']),
                metrics=value['metrics'])


def eligibility(candidate,baseline):
    c,b=candidate['metrics'],baseline['metrics'];reasons=[]
    def mean(key):
        if c[key]['mean'] is None or b[key]['mean'] is None:reasons.append('Missing '+key);return None,None
        return c[key]['mean'],b[key]['mean']
    for key in ('mission_success_pct','nav_success_pct'):
        x,y=mean(key)
        if x is not None and x<y:reasons.append(key+' inferior')
    for key,increase in [('amcl_rmse_m',.005),('slew_limit_pct',1.),('saturation_pct',.1)]:
        x,y=mean(key)
        if x is not None and x>y+increase:reasons.append(key+' exceeds predefined margin')
    x,y=mean('wheel_rmse_rad_s')
    if x is not None and not x<=.95*y:reasons.append('Wheel RMSE improvement below 5%')
    peak=c['torque_peak_nm']['p95']
    if peak is None or peak>2.0+1e-9:reasons.append('Torque operating limit violation/missing')
    if candidate['episodes']!=baseline['episodes']:reasons.append('Unequal held-out episode counts')
    return reasons


def main(argv=None):
    # Một luồng duy nhất: reuse log, khóa gain rồi tối đa hai lượt PID mới.
    from rl_training.calibrate_pid_measured import main as bounded_main
    return bounded_main(argv)


if __name__=='__main__':raise SystemExit(main())
