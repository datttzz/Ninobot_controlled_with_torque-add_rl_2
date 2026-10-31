"""Clock xác định, cùng freshness cho PI/PID; không cần Gazebo."""
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nino_control.effort_drive import EffortDrive


@pytest.fixture(params=[0.,.002])
def rig(request):
    rclpy.init(args=['--ros-args','-p',f'velocity_kd:={request.param}'],domain_id=85)
    node=EffortDrive()
    class Clock:
        ns=10_000_000_000
        def now(self):return Time(nanoseconds=self.ns)
    clock=Clock();node.get_clock=lambda:clock
    node.last_control_ns=clock.ns-1_000_000;node.last_odom_publish_ns=0
    published=[];node._publish_odometry=lambda *args:published.append(args)
    def joint(v=1.,stamp_ns=None):
        m=JointState(name=[node.left_joint,node.right_joint],velocity=[v,v])
        ns=clock.ns if stamp_ns is None else stamp_ns
        m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,1_000_000_000)
        node._joint_state_callback(m)
    yield node,clock,joint,published
    node.destroy_node();rclpy.shutdown()


def test_stale_duplicate_blocks_pi_pid_residual_and_no_fake_odom(rig):
    node,clock,joint,published=rig;joint();stamp=node.last_joint_stamp_ns
    command=Twist();command.linear.x=.1;node._cmd_vel_callback(command)
    node._control_update();clock.ns+=1_000_000;node._control_update()
    before=(node.x,node.y,node.yaw);count=len(published)
    node.error_integral=[1.,1.];clock.ns+=251_000_000
    joint(stamp_ns=stamp) # Cùng header không làm dữ liệu cũ trở nên tươi.
    node._control_update()
    assert not node.have_wheel_state and node.last_joint_stamp_ns==stamp
    assert node.error_integral==[0.,0.]
    assert (node.x,node.y,node.yaw)==before and len(published)==count
    assert all(d.value==0. for d in node.derivatives)
    clock.ns+=1_000_000;joint();node._control_update()
    assert node.have_wheel_state and (node.x,node.y,node.yaw)==before
    clock.ns+=1_000_000;node._control_update()
    assert 0<node.x-before[0]<.001 # Chỉ dt mới, không bắc qua mất dữ liệu.


def test_nonfinite_backwards_and_gap_reset_shared_state(rig):
    node,clock,joint,_=rig;joint();stamp=node.last_joint_stamp_ns
    node.error_integral=[1.,1.];joint(float('nan'))
    assert not node.have_wheel_state and node.error_integral==[0.,0.]
    joint(stamp_ns=stamp-1_000_000)
    assert not node.have_wheel_state and node.last_joint_stamp_ns==stamp
    clock.ns+=300_000_000;joint()
    assert node.have_wheel_state and node.odometry_resume_pending
    assert all(d.value==0. for d in node.derivatives)


def test_direct_torque_legacy_command_preserved_but_odom_not_invented(rig):
    node,clock,_,published=rig
    node._torque_callback(Float64MultiArray(data=[.2,.2]));node._control_update()
    assert any(v>0 for v in node.applied_effort)
    assert not node.have_wheel_state and not published
