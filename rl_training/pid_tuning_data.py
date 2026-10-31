"""Đo/chọn PID theo AMCL, IMU và encoder; đọc lại được log lịch sử đủ nguồn."""
import json
import math
from pathlib import Path

import numpy as np
import yaml
from benchmark_pi_pid_sac import episode_metrics,read_rows,rms,stats,array
from rl_training.ablation_suite import ROOT,sha,geometry_hash,reference_data
from rl_training.pose_measurements import signal_timing

GAIN_KEYS=('velocity_kp','velocity_ki','velocity_kd','derivative_filter_seconds')


def compatibility(folder,expected_mode="PID"):
    """Điều kiện vật lý/tham chiếu; không dùng nhãn pass của báo cáo lịch sử."""
    folder=Path(folder);metadata=json.loads((folder/'metadata.json').read_text())
    config=yaml.safe_load((folder/'config.yaml').read_text())
    current=yaml.safe_load((ROOT/'rl_training/config/sac.yaml').read_text())
    expected=yaml.safe_load((ROOT/'src/nino_control/config/effort_drive.yaml').read_text())['effort_drive']['ros__parameters']
    controller=metadata.get('controller') or yaml.safe_load((folder/'controller.yaml').read_text())['effort_drive']['ros__parameters']
    reasons=[]
    if metadata.get('controller_mode')!=expected_mode or metadata.get('model'):reasons.append('Controller mode differs or SAC is enabled')
    for key,value in expected.items():
        if key not in GAIN_KEYS and key!='enable_residual' and controller.get(key)!=value:
            reasons.append('Controller geometry/limit differs: '+key)
    for key in ('goal_x','goal_y','max_path_deviation','step_seconds'):
        if config['environment'].get(key)!=current['environment'].get(key):reasons.append('Environment differs: '+key)
    if metadata.get('nav2_snapshot_sha256')!=sha(ROOT/'src/nino_navigation/config/nav2.yaml'):
        reasons.append('Nav2 snapshot missing/different')
    for path in (ROOT/'maps/hall_01.yaml',ROOT/'maps/hall_01.pgm',ROOT/'src/nino_description/worlds/long_hall.sdf'):
        if metadata.get('sources',{}).get(str(path))!=sha(path):reasons.append('Plant/map evidence missing/different: '+path.name)
    reference=json.loads((folder.parent/'reference_path.json').read_text())
    if geometry_hash(reference)!=geometry_hash(reference_data(None,current)):reasons.append('Reference geometry differs')
    return dict(compatible=not reasons,reasons=reasons,
                measurement_basis=metadata.get('measurement_basis','legacy_reprocessed_from_raw_sources'),
                controller_revision_matches=metadata.get('sources',{}).get(str(ROOT/'src/nino_control/nino_control/effort_drive.py'))==sha(ROOT/'src/nino_control/nino_control/effort_drive.py'))


def measurement(folder):
    folder=Path(folder);reference=json.loads((folder.parent/'reference_path.json').read_text())
    episodes=[]
    for session in sorted(folder.glob('episode_*')):
        raw=session/'telemetry.jsonl'
        if not raw.exists():continue
        rows=read_rows(raw)
        if not rows or rows[-1].get('outcome') is None:continue
        result=episode_metrics(rows,reference);m=result['metrics']
        imu=signal_timing(rows,'imu_stamp_s');control=signal_timing(rows,'control_stamp_s')
        acceleration=array(rows,'acceleration',3);gyro=array(rows,'gyro',3)
        roll=array(rows,'roll');pitch=array(rows,'pitch')
        wheel=array(rows,'wheel_velocity',2);target=array(rows,'wheel_target',2)
        def finite_peak(values):
            values=np.asarray(values);values=values[np.isfinite(values)]
            return float(np.max(np.abs(values))) if len(values) else None
        invalid_feedback=sum(r.get('feedback_valid') is False for r in rows)
        missing_feedback=sum('feedback_valid' not in r for r in rows)
        m.update(lateral_accel_rms_m_s2=rms(acceleration[:,1],imu['weights']),
                 roll_angle_rms_rad=rms(roll,imu['weights']),roll_angle_peak_rad=finite_peak(roll),
                 pitch_angle_peak_rad=finite_peak(pitch),
                 measured_wheel_peak_rad_s=finite_peak(wheel),target_wheel_peak_rad_s=finite_peak(target))
        data_errors=[]
        for name,values in [('IMU acceleration',acceleration),('IMU gyro',gyro),('roll',roll),('pitch',pitch),('wheel',wheel),('target',target)]:
            if not np.isfinite(values).all():data_errors.append('Missing/nonfinite '+name)
        if invalid_feedback:data_errors.append('Invalid feedback samples: '+str(invalid_feedback))
        if any(np.any(np.abs(np.asarray(r.get('residual_applied',[float('nan')]*2)))>1e-12) for r in rows):
            data_errors.append('Nonzero residual')
        if any(not np.isfinite(np.asarray(r.get('residual_applied',[float('nan')]*2))).all() for r in rows):
            data_errors.append('Missing residual measurement')
        episodes.append(dict(session=str(session),**result['task_status'],outcome=result['outcome'],metrics=m,
             failure_reasons=result['mission_failure_reasons'],coverage=result['coverage'],duration_s=result['duration_s'],
             terminal_measurement=result['terminal_measurement'],data_errors=data_errors,
             missing_feedback_diagnostic_samples=missing_feedback,raw_sha256=sha(raw),
             sources=dict(pose='AMCL map->base_footprint estimate',wheel='encoder/controller JointState feedback',
                          lateral_vibration='body IMU ay after quaternion gravity subtraction',torque='controller command after limiter')))
    if not episodes:raise ValueError('No complete terminal telemetry: '+str(folder))
    return dict(episodes=episodes,metrics={k:stats(e['metrics'].get(k) for e in episodes) for k in episodes[0]['metrics']})


def rejection_reasons(value,require_feedback=False):
    reasons=[]
    for i,e in enumerate(value['episodes'],1):
        m=e['metrics']
        if e['mission_pass'] is not True:reasons.append(f'episode {i}: AMCL task failed/unknown: '+', '.join(e['failure_reasons']))
        reasons.extend(f'episode {i}: '+reason for reason in e.get('data_errors',[]))
        if require_feedback and e.get('missing_feedback_diagnostic_samples',0):reasons.append(f'episode {i}: missing feedback diagnostics')
        for key,limit in [('roll_angle_peak_rad',.5),('pitch_angle_peak_rad',.5),('measured_wheel_peak_rad_s',24.),('target_wheel_peak_rad_s',12.),('torque_peak_nm',2.)]:
            v=m.get(key)
            if v is None or not math.isfinite(v) or v>limit+1e-9:reasons.append(f'episode {i}: {key} missing/exceeds {limit}')
        for source in ('amcl','az','controller'):
            if e['coverage'].get(source,0)<.99:reasons.append(f'episode {i}: missing {source} measurement coverage')
        for key in ('wheel_rmse_rad_s','amcl_rmse_m','lateral_accel_rms_m_s2'):
            if m.get(key) is None or not math.isfinite(m[key]):reasons.append(f'episode {i}: missing '+key)
    return reasons


def ranking(value):
    """Thứ tự từ điển, không điểm tổng hợp hoặc yêu cầu slew=0."""
    return tuple(value['metrics'][k]['mean'] if value['metrics'][k]['mean'] is not None else math.inf for k in
                 ('wheel_rmse_rad_s','amcl_rmse_m','lateral_accel_rms_m_s2'))


def combine(*values):
    episodes=[e for value in values for e in value['episodes']]
    return dict(episodes=episodes,metrics={k:stats(e['metrics'].get(k) for e in episodes) for k in episodes[0]['metrics']})
