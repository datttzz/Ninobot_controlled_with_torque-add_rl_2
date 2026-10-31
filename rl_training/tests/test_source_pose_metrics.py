from copy import deepcopy
import math
import numpy as np
import pytest
from benchmark_pi_pid_sac import episode_metrics
from rl_training.pose_measurements import source_pose,terminal_pose,resample_signal
from rl_training.tests.test_fixed3_metrics import rows,REF


def test_source_weights_ignore_row_clock_offset():
    data=rows();data[-1]['outcome']='SUCCEEDED'
    for row in data:row['simulation_time']+=10
    result=episode_metrics(data,REF)
    assert result['metrics']['amcl_rmse_m']==pytest.approx(math.sqrt(.0325))
    assert result['task_status']['mission_pass'] is True


def test_duplicate_gap_backwards_and_angle_wrap():
    data=rows();data.insert(1,deepcopy(data[0]));s=source_pose(data,'amcl','amcl_stamp_s')
    assert s['duplicates']==1 and s['weights'].sum()==pytest.approx(.04)
    data[-1]['amcl_stamp_s']=-1.
    with pytest.raises(ValueError,match='backwards'):source_pose(data,'amcl','amcl_stamp_s')
    data=rows();data[-1]['amcl_stamp_s']=1.
    assert source_pose(data,'amcl','amcl_stamp_s')['coverage']<.1
    data=rows();data[-1]['terminal_clock_s']=.025
    data[1]['amcl_yaw']=math.radians(179);data[2]['amcl_yaw']=math.radians(-179)
    p,reason=terminal_pose(data,'amcl','amcl_stamp_s')
    assert reason is None and abs(abs(p[2])-math.pi)<.02


def test_terminal_needs_fixed_clock_bracket_not_best_pose():
    data=rows();data[-1]['outcome']='SUCCEEDED';data[-1]['terminal_clock_s']=.05
    result=episode_metrics(data,REF)
    assert result['metrics']['amcl_goal_xy_m'] is None
    support=deepcopy(data[-1]);support.update(amcl_stamp_s=.06,amcl_x=.4)
    data[-1]['terminal_support']=[support]
    result=episode_metrics(data,REF)
    assert result['metrics']['amcl_goal_xy_m']==pytest.approx(math.hypot(.3,.2))
    assert result['metrics']['mission_success_pct']==0.
    assert result['metrics']['amcl_max_m']==.2


def test_relative_clock_is_not_absolute_terminal():
    data=rows();data[-1].pop('simulation_clock_s')
    assert terminal_pose(data,'amcl','amcl_stamp_s')[0] is None


def test_signal_interpolation_rejects_gap_backwards_and_extrapolation():
    assert resample_signal([0,.02,.02,.04],[0,1,2,4],[.03])[0]==pytest.approx(3)
    assert np.isnan(resample_signal([0,.5],[0,1],[-.1,.25,.6])).all()
    with pytest.raises(ValueError):resample_signal([.1,0],[0,1],[.05])
