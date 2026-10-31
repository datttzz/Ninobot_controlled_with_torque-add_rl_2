"""Điều kiện độc lập, ngưỡng chính xác và metadata pose; không cần ROS."""
from types import SimpleNamespace as NS
import math
import pytest
from rl_training.mission_measurement import mission_conditions, pose_evidence


def test_observed_violation_remains_failure_when_terminal_is_missing():
    result=mission_conditions('SUCCEEDED',None,None,.15,None,None,.206)
    assert result['amcl_task_pass'] is None
    assert result['gt_task_pass'] is False and result['full_pass'] is False
    assert mission_conditions('FAILED',None,None,None,None,None,None)['full_pass'] is False


def test_independent_checks_are_not_nav2_status():
    result=mission_conditions('SUCCEEDED',.20,.20,.15,.30,.20,.15)
    assert result==dict(nav2_succeeded=True,amcl_task_pass=True,gt_task_pass=False,full_pass=False)
    result=mission_conditions('FAILED',.20,.20,.15,.20,.20,.15)
    assert result['amcl_task_pass'] and result['gt_task_pass'] and not result['full_pass']


def test_exact_inclusive_threshold_without_rounding():
    assert mission_conditions('SUCCEEDED',.25,.25,.20,.25,-.25,.20)['full_pass']
    assert not mission_conditions('SUCCEEDED',.25,.25,.20,math.nextafter(.25,1),.25,.20)['full_pass']
    assert not mission_conditions('SUCCEEDED',.25,.25,.20,.25,.256693,.20)['gt_task_pass']


@pytest.mark.parametrize('missing',[None,float('nan'),float('inf')])
def test_missing_ground_truth_not_replaced_by_amcl(missing):
    result=mission_conditions('SUCCEEDED',.1,.1,.1,missing,.1,.1)
    assert result['gt_task_pass'] is None and result['full_pass'] is None


def test_pose_evidence_preserves_frames_quaternions_and_state():
    q=NS(x=0.,y=0.,z=.1,w=.9949874371)
    transform=NS(header=NS(frame_id='map'),child_frame_id='base_footprint',transform=NS(rotation=q))
    gt=NS(child_frame_id='base_footprint',pose=NS(pose=NS(position=NS(z=.002),orientation=q)))
    result=pose_evidence(transform,gt,9.5,dict(state='STOPPING'))
    assert result['episode_state']=='STOPPING' and result['simulation_clock_s']==9.5
    assert result['amcl_frame']=='map' and result['ground_truth_child_frame']=='base_footprint'
    assert result['ground_truth_quaternion']==[0.,0.,.1,.9949874371]
