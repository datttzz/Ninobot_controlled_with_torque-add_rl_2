"""Điều kiện nhiệm vụ theo định vị; không đồng nhất với outcome của Nav2."""
import math

XY_LIMIT = 0.25
YAW_LIMIT = 0.25
DEVIATION_LIMIT = 0.20
MEASUREMENT_BASIS = 'AMCL_IMU_ENCODER_v1'


def mission_conditions(outcome, amcl_xy, amcl_yaw, amcl_max):
    """Terminal sau STOPPING; max từ FOLLOWING. So <=, không nới epsilon."""
    values=(amcl_xy,amcl_yaw,amcl_max)
    limits=(XY_LIMIT,YAW_LIMIT,DEVIATION_LIMIT)
    if any(v is not None and math.isfinite(v) and abs(v)>limit for v,limit in zip(values,limits)):
        amcl=False
    elif any(v is None or not math.isfinite(v) for v in values):
        amcl=None
    else:
        amcl=True
    if outcome in ('FAILED','TIME_LIMIT') or amcl is False:
        mission=False
    elif outcome!='SUCCEEDED' or amcl is None:
        mission=None
    else:
        mission=True
    return dict(nav2_succeeded=outcome=='SUCCEEDED',amcl_task_pass=amcl,mission_pass=mission)


def pose_evidence(transform, clock_s, state):
    """TF gốc tại base_footprint; pose là ước lượng định vị."""
    q=transform.transform.rotation
    return dict(simulation_clock_s=clock_s,episode_state=state.get('state'),
                amcl_frame=transform.header.frame_id,amcl_child_frame=transform.child_frame_id,
                amcl_quaternion=[q.x,q.y,q.z,q.w],pose_source='TF AMCL + encoder odometry estimate')
