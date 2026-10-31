"""Ngưỡng AMCL chính xác và outcome Nav2 độc lập."""
from types import SimpleNamespace as NS
import math
import pytest
from rl_training.mission_measurement import mission_conditions,pose_evidence


def test_nav_and_amcl_are_separate():
    assert mission_conditions('SUCCEEDED',.3,.1,.1)==dict(nav2_succeeded=True,amcl_task_pass=False,mission_pass=False)
    assert mission_conditions('FAILED',.1,.1,.1)==dict(nav2_succeeded=False,amcl_task_pass=True,mission_pass=False)


def test_exact_inclusive_threshold_without_rounding():
    assert mission_conditions('SUCCEEDED',.25,-.25,.20)['mission_pass']
    assert not mission_conditions('SUCCEEDED',math.nextafter(.25,1),.25,.20)['mission_pass']
    assert not mission_conditions('SUCCEEDED',.1,.251,.1)['amcl_task_pass']


@pytest.mark.parametrize('missing',[None,float('nan'),float('inf')])
def test_missing_pose_is_unknown_but_known_violation_still_fails(missing):
    assert mission_conditions('SUCCEEDED',missing,.1,.1)['mission_pass'] is None
    assert mission_conditions('SUCCEEDED',missing,.1,.206)['mission_pass'] is False


def test_pose_evidence_uses_original_tf():
    q=NS(x=0.,y=0.,z=.1,w=.9949874371)
    transform=NS(header=NS(frame_id='map'),child_frame_id='base_footprint',transform=NS(rotation=q))
    result=pose_evidence(transform,9.5,dict(state='STOPPING'))
    assert result['simulation_clock_s']==9.5 and result['amcl_frame']=='map'
    assert result['amcl_child_frame']=='base_footprint' and result['amcl_quaternion']==[q.x,q.y,q.z,q.w]
