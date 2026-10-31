"""Admission PID theo full task thật, không cho mission 0/0 thành noninferior."""
from copy import deepcopy
from rl_training.calibrate_pid_measured import CANDIDATES, rejection_reasons, ranking, combine


def sample():
    metrics=dict(roll_angle_peak_rad=.2,pitch_angle_peak_rad=.1,measured_wheel_peak_rad_s=14.,
                 target_wheel_peak_rad_s=12.,torque_peak_nm=2.,wheel_rmse_rad_s=1.,amcl_rmse_m=.05,
                 lateral_rate_rms_m_s=.1,roll_angle_rms_rad=.03)
    return dict(episodes=[dict(mission_pass=True,metrics=metrics,failure_reasons=[],
                 coverage=dict(amcl=1.,az=1.))],metrics={k:dict(mean=v) for k,v in metrics.items()})


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
