"""Admission PID theo full task thật, không cho mission 0/0 thành noninferior."""
from copy import deepcopy
from rl_training.calibrate_pid_measured import CANDIDATES, rejection_reasons, ranking, combine


def sample():
    metrics=dict(roll_angle_peak_rad=.2,pitch_angle_peak_rad=.1,measured_wheel_peak_rad_s=14.,
                 target_wheel_peak_rad_s=12.,torque_peak_nm=2.,wheel_rmse_rad_s=1.,amcl_rmse_m=.05,
                 lateral_accel_rms_m_s2=.1,roll_angle_rms_rad=.03)
    return dict(episodes=[dict(mission_pass=True,metrics=metrics,failure_reasons=[],
                 coverage=dict(amcl=1.,az=1.,controller=1.))],metrics={k:dict(mean=v) for k,v in metrics.items()})


def test_failed_amcl_cannot_qualify_on_nav_success():
    value=sample();assert not rejection_reasons(value)
    value['episodes'][0]['mission_pass']=False
    value['episodes'][0]['failure_reasons']=['amcl_goal_xy_m=0.3 > 0.25']
    assert 'AMCL task failed' in rejection_reasons(value)[0]


def test_limiter_contact_is_not_unsafe_unless_actual_limit_exceeded():
    value=sample();value['episodes'][0]['metrics']['slew_limit_pct']=80.
    assert not rejection_reasons(value)
    value['episodes'][0]['metrics']['torque_peak_nm']=2.01
    assert any('torque_peak' in r for r in rejection_reasons(value))


def test_lexicographic_selection_no_weighted_score_or_hidden_superiority():
    a=sample();b=deepcopy(a);b['metrics']['wheel_rmse_rad_s']['mean']=.9
    b['metrics']['amcl_rmse_m']['mean']=.06
    assert ranking(b)<ranking(a)
    assert len(combine(a,b)['episodes'])==2
    assert len(CANDIDATES)==6 and all(c['velocity_kd']>0 for c in CANDIDATES)


def test_verification_requires_feedback_evidence_and_retains_data_errors():
    value=sample();value['episodes'][0]['missing_feedback_diagnostic_samples']=2
    assert not rejection_reasons(value)
    assert any('feedback' in r for r in rejection_reasons(value,require_feedback=True))
    value['episodes'][0]['data_errors']=['Invalid feedback samples: 1']
    assert rejection_reasons(value)


def test_reprocess_raw_legacy_without_prior_acceptance_label(tmp_path):
    import json
    import pytest
    from rl_training.pid_tuning_data import measurement
    from rl_training.tests.test_fixed3_metrics import rows,REF
    folder=tmp_path/'PID';folder.mkdir();session=folder/'episode_00001';session.mkdir()
    (tmp_path/'reference_path.json').write_text(json.dumps(REF))
    (folder/'metadata.json').write_text(json.dumps(dict(old_acceptance=False)))
    data=rows();data[-1]['outcome']='SUCCEEDED'
    for r in data:r.update(roll=0.,pitch=0.,feedback_valid=True,residual_applied=[0.,0.])
    data[1]['acceleration'][1]=2.;data[2]['acceleration'][1]=4.
    (session/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in data))
    value=measurement(folder)
    assert value['episodes'][0]['mission_pass'] is True
    assert value['metrics']['lateral_accel_rms_m_s2']['mean']==pytest.approx((13.)**.5)
    assert not rejection_reasons(value,require_feedback=True)


def test_ranking_uses_lateral_imu_after_wheel_and_amcl():
    a=sample();b=deepcopy(a);b['metrics']['lateral_accel_rms_m_s2']['mean']=.05
    assert ranking(b)<ranking(a)


def test_both_tuning_entrypoints_use_same_two_episode_runner(monkeypatch):
    from rl_training import calibrate_pid_measured,tune_pid
    calls=[]
    def run(args):calls.append(args);return 42
    monkeypatch.setattr(calibrate_pid_measured,'main',run)
    assert tune_pid.main(['--budget-minutes','30'])==42
    assert calls==[['--budget-minutes','30']]
