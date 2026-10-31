"""Phép đo pose AMCL và terminal theo timestamp nguồn, không ngoại suy."""
import math
import numpy as np
from .localization_math import interpolate_pose

# TF/odom và telemetry ~50 Hz: tối đa 5 chu kỳ logger.
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


def signal_timing(rows,stamp_key,max_gap=POSE_MAX_GAP_S):
    """Trọng số cho IMU/controller/odom từ timestamp nguồn, không dùng giờ nhận."""
    timing=[dict(time=row.get(stamp_key),v_x=0.,v_y=0.,v_yaw=0.) for row in rows]
    return source_pose(timing,'v','time',max_gap)


def resample_signal(times,values,targets,max_gap=POSE_MAX_GAP_S):
    """Nội suy tín hiệu tuyến tính cho plot/window, không bắc gap hoặc ngoại suy."""
    times=np.asarray(times,dtype=float);values=np.asarray(values,dtype=float)
    targets=np.asarray(targets,dtype=float);result=np.full(len(targets),np.nan)
    finite_time=times[np.isfinite(times)]
    if np.any(np.diff(finite_time)<0):raise ValueError('Signal timestamp moved backwards')
    good=np.isfinite(times)&np.isfinite(values)
    if good.sum()<2:return result
    t,v=times[good],values[good]
    _,rev=np.unique(t[::-1],return_index=True);keep=np.sort(len(t)-1-rev);t,v=t[keep],v[keep]
    if len(t)<2:return result
    right=np.minimum(np.searchsorted(t,targets),len(t)-1);left=np.maximum(right-1,0)
    exact=(targets==t[right])|(targets==t[left])
    valid=np.isfinite(targets)&(targets>=t[0])&(targets<=t[-1])&((t[right]-t[left]<=max_gap)|exact)
    result[valid]=np.interp(targets[valid],t,v)
    return result
