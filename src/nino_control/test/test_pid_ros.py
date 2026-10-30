"""Vận tốc đo tạo D có dấu hãm; Kd=0 giữ phép tính PI hiện tại."""
from math import isclose
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nino_control.effort_drive import EffortDrive


def test_pid_uses_measurement_derivative_and_preserves_limits():
    rclpy.init(args=['--ros-args', '-p', 'velocity_kd:=0.01',
                    '-p', 'derivative_filter_seconds:=0.05'], domain_id=85)
    node = EffortDrive()
    try:
        origin = node.get_clock().now().nanoseconds-20000000
        def joint(v, ns):
            message = JointState(name=[node.left_joint, node.right_joint], velocity=[v, v])
            stamp = origin + ns
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
            node._joint_state_callback(message)
        joint(0.0, 0)
        joint(0.1, 10000000)
        assert isclose(node.derivatives[0].value, 10/6, rel_tol=1e-6)
        command = Twist()
        command.linear.x = 0.1
        node._cmd_vel_callback(command)
        old = node.derivatives[0].value
        command.linear.x = 0.2
        node._cmd_vel_callback(command)
        assert node.derivatives[0].value == old  # Không derivative kick khi đổi setpoint.
        node.last_control_ns = node.get_clock().now().nanoseconds-10000000
        node._control_update()
        assert all(abs(v) <= node.max_velocity_torque for v in node.applied_effort)
        assert all(abs(v) <= 0.12 for v in node.applied_effort)  # Giữ slew limiter 10 Nm/s.
        node._cmd_vel_callback(Twist())
        assert all(d.previous is None and d.value == 0 for d in node.derivatives)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_default_pi_arithmetic_is_unchanged():
    rclpy.init(domain_id=85)
    node = EffortDrive()
    try:
        assert node.kd == 0
        node.have_wheel_state = True
        node.max_effort_rate = 1e6  # Cô lập công thức PI trong kiểm thử này.
        command = Twist()
        command.linear.x = 0.1
        node._cmd_vel_callback(command)
        node._control_update()
        for target, integral, output in zip(node.target_velocity, node.error_integral, node.applied_effort):
            assert output == node.kp*target + node.ki*integral
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pid_resets_integral_and_filter_for_invalid_or_old_measurement():
    rclpy.init(args=['--ros-args','-p','velocity_kd:=0.001'],domain_id=85)
    node=EffortDrive()
    try:
        def message(v,offset):
            m=JointState(name=[node.left_joint,node.right_joint],velocity=[v,v])
            ns=node.get_clock().now().nanoseconds+int(offset*1e9)
            m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,1000000000)
            return m
        node._joint_state_callback(message(1.,-.02))
        node.error_integral=[1.,1.]
        node._joint_state_callback(message(float('nan'),-.01))
        assert node.error_integral==[0.,0.]
        assert all(d.previous is None and d.value==0 for d in node.derivatives)
        assert not node.have_wheel_state
        node.error_integral=[1.,1.]
        node._joint_state_callback(message(1.,-1.))
        assert node.error_integral==[0.,0.] and not node.have_wheel_state
        node._joint_state_callback(message(1.,-.02))
        node.error_integral=[1.,1.]
        node._joint_state_callback(message(1.,-.1)) # Backwards stamp, though wall age < 0.25s.
        assert node.error_integral==[0.,0.] and not node.have_wheel_state
    finally:
        node.destroy_node();rclpy.shutdown()
