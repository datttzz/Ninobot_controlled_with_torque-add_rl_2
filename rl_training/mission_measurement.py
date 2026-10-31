"""Kiểm tra nhiệm vụ độc lập; không đồng nhất Nav2 thành công với pose thật."""
import math

XY_LIMIT = 0.25
YAW_LIMIT = 0.25
DEVIATION_LIMIT = 0.20


def mission_conditions(outcome, amcl_xy, amcl_yaw, amcl_max, gt_xy, gt_yaw, gt_max):
    """So sánh <= trực tiếp bằng float64, không làm tròn/nới epsilon khi xét pass.

    Pose tại terminal sau khi EpisodeManager xác nhận dừng. Lệch đường là max
    từ lúc bắt đầu FOLLOWING tới terminal; không có mẫu reset/khởi tạo.
    AMCL ở đây là TF map->base_footprint (AMCL kết hợp odometry), không phải
    riêng message /amcl_pose. GT là pose model vật lý trong world đã đăng ký.
    """
    def independent(xy, yaw, maximum):
        values = (xy, yaw, maximum)
        # Một vi phạm đã quan sát vẫn là fail dù thiếu phép đo điều kiện khác.
        for value, limit in zip(values, (XY_LIMIT, YAW_LIMIT, DEVIATION_LIMIT)):
            if value is not None and math.isfinite(value) and abs(value) > limit:
                return False
        if any(v is None or not math.isfinite(v) for v in values):
            return None
        return xy <= XY_LIMIT and abs(yaw) <= YAW_LIMIT and maximum <= DEVIATION_LIMIT

    amcl = independent(amcl_xy, amcl_yaw, amcl_max)
    gt = independent(gt_xy, gt_yaw, gt_max)
    if outcome in ('FAILED', 'TIME_LIMIT') or amcl is False or gt is False:
        full = False
    else:
        full = None if amcl is None or gt is None or outcome != 'SUCCEEDED' else amcl and gt
    return dict(nav2_succeeded=outcome == 'SUCCEEDED', amcl_task_pass=amcl,
                gt_task_pass=gt, full_pass=full)


def pose_evidence(transform, ground_truth, clock_s, state):
    """Lưu bằng chứng thụ động; không dùng để sửa pose hoặc điều khiển."""
    tq = transform.transform.rotation
    g = ground_truth.pose.pose
    return dict(simulation_clock_s=clock_s, episode_state=state.get('state'),
                amcl_frame=transform.header.frame_id,
                amcl_child_frame=transform.child_frame_id,
                amcl_quaternion=[tq.x, tq.y, tq.z, tq.w],
                ground_truth_child_frame=ground_truth.child_frame_id,
                ground_truth_z=g.position.z,
                ground_truth_quaternion=[g.orientation.x, g.orientation.y,
                                         g.orientation.z, g.orientation.w])
