from copy import deepcopy
import math
import numpy as np
import pytest
from benchmark_pi_pid_sac import episode_metrics
from rl_training.pose_measurements import source_pose,terminal_pose,synchronized_comparison
from rl_training.tests.test_fixed3_metrics import rows,REF


def test_default_missing_registration_is_na_but_amcl_and_nav_remain():
    data=rows();data[-1]['outcome']='SUCCEEDED';result=episode_metrics(data,REF)
    assert result['metrics']['gt_rmse_m'] is None
    assert result['metrics']['amcl_rmse_m'] is not None
    assert result['task_status']['nav2_succeeded'] is True
    assert result['task_status']['amcl_task_pass'] is True
    assert result['task_status']['gt_task_pass'] is None
    assert result['amcl_gt_comparison']['rmse_xy_m'] is None


def test_shifted_source_stamps_compare_at_shared_time_not_row():
    data=[]
    for t in np.arange(10.,10.101,.02):
        data.append(dict(amcl_stamp_s=t,amcl_x=t,amcl_y=0.,amcl_yaw=0.,
                         gt_stamp_s=t-.01,ground_truth_x=t-.01,ground_truth_y=0.,ground_truth_yaw=0.))
    comparison=synchronized_comparison(data,[0,0,0])
    assert comparison['samples']>=3 and comparison['rmse_xy_m']<1e-12
    assert data[0]['amcl_x']!=data[0]['ground_truth_x']


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


def test_terminal_uses_fixed_event_with_support_not_best_pose():
    data=rows();data[-1]['outcome']='SUCCEEDED';data[-1]['terminal_clock_s']=.05
    result=episode_metrics(data,REF,map_to_world=[0,0,0])
    assert result['metrics']['amcl_goal_xy_m'] is None
    assert 'not bracketed' in result['missing_reasons']['amcl_goal_xy_m']
    support=deepcopy(data[-1]);support.update(amcl_stamp_s=.06,gt_stamp_s=.06,amcl_x=.4,ground_truth_x=.4)
    data[-1]['terminal_support']=[support]
    result=episode_metrics(data,REF,map_to_world=[0,0,0])
    assert result['metrics']['amcl_goal_xy_m']==pytest.approx(math.hypot(.3,.2))
    assert result['metrics']['mission_success_pct']==0.
    assert result['metrics']['amcl_max_m']==.2 # Không thêm mẫu support sau terminal vào trajectory metrics.


def test_missing_absolute_terminal_never_substitutes_elapsed_time():
    data=rows();data[-1].pop('simulation_clock_s')
    assert terminal_pose(data,'amcl','amcl_stamp_s')[0] is None
