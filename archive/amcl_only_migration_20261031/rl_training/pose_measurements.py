"""Metric mỗi nguồn và so AMCL–GT đồng bộ được giữ riêng."""
import math
import numpy as np
from .localization_math import compose,interpolate_pose,wrap

# TF/odom ghi ~50 Hz, GT ~100 Hz, telemetry 50 Hz: tối đa 5 chu kỳ logger.
# Không ngoại suy hoặc bắc qua một khoảng mất dữ liệu dài hơn 0.10 s.
POSE_MAX_GAP_S = .10


def source_pose(rows,prefix,stamp_key,max_gap=POSE_MAX_GAP_S):
    samples=[];indices=[];stamps=[]
    for i,row in enumerate(rows):
        try:
            stamp=float(row[stamp_key])
            if math.isfinite(stamp):stamps.append(stamp)
            values=[float(row[prefix+'_'+k]) for k in ('x','y','yaw')]
            if math.isfinite(stamp) and all(math.isfinite(v) for v in values):
                samples.append([stamp,*values]);indices.append(i)
        except (KeyError,TypeError,ValueError):continue
    if np.any(np.diff(stamps)<0):raise ValueError(stamp_key+' moved backwards')
    weights=np.zeros(len(rows));valid=np.zeros(len(rows),dtype=bool)
    if not samples:
        return dict(samples=np.empty((0,4)),weights=weights,valid=valid,coverage=0.,complete=False,
                    reason='Missing finite pose/source timestamp: '+stamp_key,duplicates=0)
    data=np.array(samples);_,rev=np.unique(data[::-1,0],return_index=True)
    keep=np.sort(len(data)-1-rev);unique=data[keep];indices=np.array(indices)[keep]
    delta=np.diff(unique[:,0],prepend=unique[0,0]);covered=np.where(delta<=max_gap,delta,0.)
    weights[indices]=covered;valid[indices]=True
    duration=unique[-1,0]-unique[0,0]
    coverage=float(covered.sum()/duration) if duration>0 else 0.
    return dict(samples=unique,weights=weights,valid=valid,coverage=coverage,
                complete=len(samples)==len(rows) and coverage>=1-1e-9,
                reason=None if len(unique)>=2 else 'Insufficient unique source timestamps: '+stamp_key,
                duplicates=len(data)-len(unique))


def terminal_pose(rows,prefix,stamp_key,max_gap=POSE_MAX_GAP_S):
    last=rows[-1];target=last.get('terminal_clock_s',last.get('simulation_clock_s'))
    if target is None or not math.isfinite(target):
        return None,'Missing absolute ROS terminal timestamp; relative simulation_time is not a ROS stamp'
    support=last.get('terminal_support',[])
    data=source_pose(rows+support,prefix,stamp_key,max_gap)['samples']
    if len(data)<2:return None,'Missing source poses needed for terminal interpolation'
    value=interpolate_pose(data,[target],max_gap=max_gap)[0]
    if not np.isfinite(value).all():
        return None,'Terminal not bracketed by source timestamps within max-gap; no extrapolation'
    return value,None


def synchronized_comparison(rows,map_to_world,max_gap=POSE_MAX_GAP_S):
    """Dùng timestamp GT trong giao hai nguồn; không coi cùng hàng là cùng thời điểm."""
    result=dict(max_gap_s=max_gap,time_basis='absolute ROS source stamps',samples=0,coverage=0.,
                rmse_xy_m=None,rmse_yaw_rad=None,reason=None)
    if map_to_world is None:
        result['reason']='Missing verified map→world registration';return result
    a=source_pose(rows,'amcl','amcl_stamp_s',max_gap)['samples']
    g=source_pose(rows,'ground_truth','gt_stamp_s',max_gap)['samples']
    if len(a)<2 or len(g)<2:
        result['reason']='Missing finite AMCL/GT source timestamps';return result
    times=g[:,0];ap=compose(map_to_world,interpolate_pose(a,times,max_gap=max_gap))
    gp=interpolate_pose(g,times,max_gap=max_gap);valid=np.isfinite(ap).all(axis=1)&np.isfinite(gp).all(axis=1)
    error=ap-gp;error[:,2]=wrap(error[:,2]);delta=np.diff(times,prepend=times[0])
    good=valid&np.r_[False,valid[:-1]]&(delta<=max_gap);weights=np.where(good,delta,0.)
    overlap=max(0,min(a[-1,0],g[-1,0])-max(a[0,0],g[0,0]))
    result.update(samples=int(valid.sum()),coverage=float(weights.sum()/overlap) if overlap else 0.,
                  t_ros_s=times[valid].tolist(),error_x_m=error[valid,0].tolist(),
                  error_y_m=error[valid,1].tolist(),error_yaw_rad=error[valid,2].tolist())
    if weights.sum()>0:
        result['rmse_xy_m']=float(np.sqrt(np.sum(np.where(valid,np.sum(error[:,:2]**2,axis=1),0.)*weights)/weights.sum()))
        result['rmse_yaw_rad']=float(np.sqrt(np.sum(np.where(valid,error[:,2]**2,0.)*weights)/weights.sum()))
    else:result['reason']='Insufficient common timed coverage; no extrapolation/gap filling'
    return result
