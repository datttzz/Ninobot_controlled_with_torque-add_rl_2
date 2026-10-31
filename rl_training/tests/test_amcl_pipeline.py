"""Luồng AMCL-only thật ở observe/logger, không cần mở Gazebo."""
from pathlib import Path
from types import SimpleNamespace as NS
from time import monotonic
import sys
import json
import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'rl_training'))
from ros_env import NinoResidualEnv,OBSERVATIONS
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,JointState
from std_msgs.msg import Float64MultiArray,String
from rclpy.time import Time


def fixture():
    transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_footprint'
    transform.header.stamp=Time(seconds=1.).to_msg()
    transform.transform.translation.x=.5;transform.transform.translation.y=.1;transform.transform.rotation.w=1.
    imu=Imu();imu.header.stamp=Time(seconds=1.).to_msg();imu.orientation.w=1.
    imu.linear_acceleration.z=9.80665
    imu.angular_velocity.x=.3;imu.angular_velocity.y=-.6;imu.angular_velocity.z=.9
    odom=Odometry();odom.header.stamp=Time(seconds=1.).to_msg()
    odom.twist.twist.linear.x=.2;odom.twist.twist.angular.z=.15
    joints=JointState(name=['left_wheel_joint','right_wheel_joint'],velocity=[2.,3.])
    cmd=Twist();cmd.linear.x=.25;cmd.angular.z=.1
    status=dict(stamp_s=1.,wheel_target=[2.,3.],wheel_velocity=[2.,3.],saturated=[False,False],
                slew_limited=[False,False],control_cycles=1000,slew_cycles=[0,0],saturation_cycles=[0,0],
                derivative_alpha=[0.,0.],feedback_valid=True,feedback_status='fresh',feedback_stamp_s=1.,
                direct_torque_without_feedback=False)
    data=dict(imu=imu,odom=odom,joints=joints,cmd=cmd,
              base=Float64MultiArray(data=[.2,.3]),total=Float64MultiArray(data=[.2,.33]),
              residual=Float64MultiArray(data=[0.,.03]),control_status=String(data=json.dumps(status)))
    reference=NS(poses=[NS(pose=NS(position=NS(x=x,y=0.))) for x in (0.,1.)])
    state=dict(state='FOLLOWING',planning_requests=1,path_sha256='same')
    bridge=NS(snapshot=lambda:(data,{k:monotonic() for k in data},reference,'same',state),
              tf=NS(lookup_transform=lambda *args:transform),get_clock=lambda:NS(now=lambda:Time(seconds=1.01)))
    env=object.__new__(NinoResidualEnv);env.bridge=bridge;env.executor_error=None
    env.cfg=dict(sensor_timeout=1.,max_path_deviation=.2,residual_limit=.3,goal_x=1.,goal_y=0.,max_episode_seconds=90)
    env.observation_space=NS(shape=(23,));env.seed_label=1
    env.last_action=np.array([.1,-.2]);return env


def test_observation_order_units_and_scaling_unchanged():
    env=fixture();observation,info=env._observe()
    expected=[.5,0.,.05,0.,0.,0.,0.,0.,.1,-.2,.3,.4,.15,.5,.1,2/12,3/12,.1,.15,.1,-.2,0.,.1]
    assert len(OBSERVATIONS)==23 and observation==pytest.approx(expected,abs=1e-7)
    assert info['body_speed_m_s']==.2 and 'estimate' in info['body_velocity_source']
    assert 'command' in info['torque_source']


def test_telemetry_and_terminal_support_need_only_amcl_and_sensors():
    env=fixture();env.start_sim=0.;env.finished=False;env.result_future=NS(done=lambda:False)
    recorded=[];env.telemetry=True;env._write_telemetry=recorded.append
    env._record_benchmark();assert len(recorded)==1
    row=recorded[0]
    assert row['amcl_x']==.5 and row['imu_stamp_s']==1. and row['odom_stamp_s']==1.
    assert row['body_speed_m_s']==.2 and row['feedback_valid'] is True
    def wait(predicate,**kwargs):assert predicate()
    env._wait=wait
    support,status=env._terminal_pose_support(1.)
    assert status=='amcl_samples_bracket_terminal' and support[0]['amcl_stamp_s']==1.
    assert not any(k.startswith(('gt_','ground_truth')) for k in row)


def test_no_pose_publisher_plugin_or_dedicated_bridge_and_cli_removed():
    bridge=yaml.safe_load((ROOT/'src/nino_description/config/bridge.yaml').read_text())
    assert {r['ros_topic_name'] for r in bridge}=={'/clock','/imu/data','/scan'}
    xacro=(ROOT/'src/nino_description/urdf/nino.urdf.xacro').read_text()
    assert 'OdometryPublisher' not in xacro
    from rl_training.ablation_suite import make_plan,group_spec
    from rl_training.tests.test_ablation_suite import args
    plan=make_plan(args());spec=group_spec(plan,plan['groups'][0],Path('/tmp/unused_amcl_only'))
    assert 'registration' not in plan and 'registration' not in spec
    assert 'map_to_world' not in spec


def test_audit_works_without_extra_pose_sources(tmp_path):
    from rl_training.audit_mission_measurements import audit_episode
    from rl_training.tests.test_fixed3_metrics import rows,REF
    data=rows();data[-1]['outcome']='SUCCEEDED'
    (tmp_path/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in data))
    result,_=audit_episode(tmp_path,REF)
    assert result['nav2_succeeded'] and result['amcl_task_pass'] and result['mission_pass']
