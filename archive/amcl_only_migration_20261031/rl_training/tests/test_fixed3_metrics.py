"""Dữ liệu có đáp án biết trước; không mở ROS/Gazebo."""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import benchmark_pi_pid_sac as metrics

REF=dict(frame_id='map',poses=[dict(x=0.,y=0.,yaw=0.),dict(x=1.,y=0.,yaw=0.)])


def rows():
    return [dict(simulation_time=t,simulation_clock_s=t,amcl_stamp_s=t,gt_stamp_s=t,imu_stamp_s=t,amcl_x=x,amcl_y=e,amcl_yaw=0.,
      ground_truth_x=x,ground_truth_y=e,ground_truth_yaw=0.,ground_truth_frame='world',
      ground_truth_speed=.2,acceleration=[0.,0.,az],gyro=[0.,0.,0.],
      wheel_velocity=[1.,1.],wheel_target=[2.,2.],base_torque=[.2,.2],
      total_torque=[.3,.3],residual_applied=[.1,.1],outcome=None,
      reference_geometry_sha256='same') for t,x,e,az in [(0.,0.,0.,0.),(.01,.5,.1,1.),(.04,1.,.2,2.)]]


def test_time_weighted_rmse_percentile_and_actuators():
    data=rows();data[-1]['outcome']='SUCCEEDED'
    result=metrics.episode_metrics(data,REF,map_to_world=[0,0,0])['metrics']
    assert result['gt_rmse_m']==pytest.approx(math.sqrt((.1**2+3*.2**2)/4))
    assert result['gt_p95_m']==.2
    assert result['wheel_rmse_rad_s']==1.
    assert result['torque_rms_nm']==pytest.approx(.3)
    assert result['sac_contribution_pct']==pytest.approx(100/3)
    assert result['mission_success_pct']==100
    assert result['completion_time_s']==.04


def test_failed_run_is_kept_and_has_no_completion_time():
    data=rows();data[-1]['outcome']='FAILED';data[-1]['amcl_y']=.3
    result=metrics.episode_metrics(data,REF,map_to_world=[0,0,0])
    assert result['outcome']=='FAILED'
    assert result['metrics']['mission_success_pct']==0
    assert result['metrics']['deviation_violation_pct']==100
    assert result['metrics']['completion_time_s'] is None
    assert result['metrics']['gt_rmse_m'] is not None


def test_jerk_filter_has_exact_known_recurrence():
    t=np.array([0.,.1,.2]);az=np.array([0.,1.,2.])
    alpha=math.exp(-math.pi)
    jerk=metrics.filtered_jerk(az,t)
    assert math.isnan(jerk[0])
    assert jerk[1]==pytest.approx(10*(1-alpha))
    assert jerk[2]==pytest.approx(10*(1-alpha**2))
    assert np.isnan(metrics.filtered_jerk([0,1,2],[0,0,.5])).all()


def test_missing_signals_and_near_zero_denominator_are_na():
    data=rows()
    for row in data:
        for key in ('ground_truth_x','ground_truth_y','ground_truth_yaw','acceleration','gyro','imu_stamp_s','total_torque'):
            row.pop(key)
        row['base_torque']=[1e-15,1e-15];row['residual_applied']=[1e-15,1e-15]
    data[-1]['outcome']='SUCCEEDED'
    result=metrics.episode_metrics(data,REF)
    for key in ('gt_rmse_m','az_rms_m_s2','jerk_rms_m_s3','torque_rms_nm','sac_contribution_pct','mission_success_pct'):
        assert result['metrics'][key] is None,key
        assert key in result['missing_reasons']
    assert result['metrics']['amcl_rmse_m'] is not None


@pytest.mark.parametrize('times',[[0,2,1],[0,1,1],[0,float('nan'),2],[0,0,0]])
def test_bad_timestamps_rejected(times):
    data=rows()
    for row,t in zip(data,times):row['simulation_time']=t
    with pytest.raises(ValueError):metrics.episode_metrics(data,REF)


def test_terminal_duplicate_has_no_extra_time_weight():
    data=rows();terminal=deepcopy(data[-1]);terminal['outcome']='SUCCEEDED'
    a=metrics.episode_metrics(data+[terminal],REF,map_to_world=[0,0,0])
    data[-1]['outcome']='SUCCEEDED';b=metrics.episode_metrics(data,REF,map_to_world=[0,0,0])
    assert a['metrics']['gt_rmse_m']==b['metrics']['gt_rmse_m']


def test_p95_between_episodes_is_distinct_from_within_episode():
    assert metrics.percentile([0,100],weights=[99,1])==0
    assert metrics.stats([0,100])['p95']==pytest.approx(95)


def test_settling_recovery_extends_beyond_local_window():
    t=np.arange(0,8,.02);x=t*.5;error=np.where(t<3.4,.1,0.01)
    az=np.where(t<3.4,2.,0.);speed=np.full(len(t),.5)
    result=metrics.window_metrics(t,x,error,az,np.zeros(len(t)),speed,np.full(len(t),.02),[1.],.5)[0]
    assert result['settling_time_s']>1 # Exit of +0.5m window is 1s after crossing.
    assert result['settling_status']=='observed'


def test_missing_frame_registration_does_not_assume_gt_matches_map():
    data=rows();data[-1]['outcome']='SUCCEEDED'
    result=metrics.episode_metrics(data,REF,map_to_world=None)
    assert result['metrics']['gt_rmse_m'] is None
    assert result['metrics']['mission_success_pct'] is None


def test_checkpoint_is_verified_from_bytes_and_metadata(tmp_path):
    import zipfile,hashlib
    model=tmp_path/'false_70000.zip'
    with zipfile.ZipFile(model,'w') as archive:archive.writestr('data',json.dumps(dict(ent_coef='auto',num_timesteps=1234)))
    digest=hashlib.sha256(model.read_bytes()).hexdigest()
    metadata=dict(model=str(model),checkpoint_evidence=dict(sha256=digest,timesteps=1234))
    assert metrics.checkpoint_evidence(metadata)['timesteps']==1234
    metadata['checkpoint_evidence']['sha256']='wrong'
    with pytest.raises(ValueError,match='hash'):metrics.checkpoint_evidence(metadata)


def test_three_group_exports_keep_failure_schema_and_metadata(tmp_path):
    import csv,hashlib,yaml,zipfile
    from rl_training.frame_registration import load_registration,ROOT
    from rl_training.tests.test_frame_registration import write_registration
    registration=load_registration(write_registration(tmp_path,[0,0,0]),ROOT/'maps/hall_01.yaml')
    inputs=[]
    for label,mode,kd in [('PI','PI',0.),('PID','PID',.001),('PI_SAC','SAC',0.)]:
        folder=tmp_path/label;folder.mkdir();inputs.append(folder)
        (tmp_path/'reference_path.json').write_text(json.dumps(REF))
        metadata=dict(controller_mode=mode,mode='evaluate' if label=='PI_SAC' else 'baseline',
                      deterministic=True,registration=registration,map=str(ROOT/'maps/hall_01.yaml'),map_to_world=[0,0,0],gains=dict(velocity_kd=kd))
        if label=='PI_SAC':
            model=tmp_path/'chosen.zip'
            with zipfile.ZipFile(model,'w') as archive:archive.writestr('data',json.dumps(dict(ent_coef='auto',num_timesteps=1234)))
            metadata.update(model=str(model),checkpoint_evidence=dict(timesteps=1234,sha256=hashlib.sha256(model.read_bytes()).hexdigest()))
        (folder/'metadata.json').write_text(json.dumps(metadata))
        (folder/'config.yaml').write_text(yaml.safe_dump(dict(environment=dict(max_path_deviation=.2))))
        episode=folder/'episode_00001';episode.mkdir();data=rows();data[-1]['outcome']='FAILED' if label=='PID' else 'SUCCEEDED'
        if label!='PI_SAC':
            for r in data:r['residual_applied']=[0.,0.]
        (episode/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in data))
    output=tmp_path/'report'
    report=metrics.build_report(*inputs,output,centers=[.5])
    assert report['groups'][1]['summary']['mission_success_pct']['mean']==0
    assert report['groups'][2]['checkpoint_proof']['timesteps']==1234
    text=(output/'report_PI_PID_SAC.html').read_text()
    assert '<svg' in text and 'https://' not in text and '<script src=' not in text.lower()
    contribution_row=next(row for row in text.split('<tr>') if 'Đóng góp |SAC|' in row)
    assert '<strong>' not in contribution_row.split('</tr>')[0]
    tex=(output/'comparison_table.tex').read_text()
    assert '\\toprule' in tex and 'Median & P95' in tex and len(tex.splitlines())>10
    summary=list(csv.DictReader((output/'summary.csv').open()))
    assert any(row['section'].startswith('4.') for row in summary)
    with pytest.raises(FileExistsError):metrics.build_report(*inputs,output,centers=[.5])


def test_csv_writes_explicit_na_with_no_zero_imputation(tmp_path):
    path=tmp_path/'na.csv'
    metrics.write_csv(path,[dict(metric='completion',mean=None,std=None,n=0,reason='no successful episode')])
    import csv
    row=next(csv.DictReader(path.open()))
    assert row['mean']=='N/A' and row['std']=='N/A' and row['n']=='0'
    assert row['reason']=='no successful episode'


def test_terminal_condition_reasons_explain_nav_success_without_mission_success():
    data=rows();data[-1]['outcome']='SUCCEEDED';data[-1]['ground_truth_x']=.5
    result=metrics.episode_metrics(data,REF,map_to_world=[0,0,0])
    assert result['metrics']['nav_success_pct']==100
    assert result['metrics']['mission_success_pct']==0
    assert any('gt_goal_xy_m' in reason for reason in result['mission_failure_reasons'])
