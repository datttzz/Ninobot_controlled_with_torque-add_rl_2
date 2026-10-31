"""SE(2) và đồng bộ pose để chẩn đoán; không thay đổi nguồn pose."""
import numpy as np


def wrap(value):
    return np.arctan2(np.sin(value), np.cos(value))


def compose(a, b):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    c,s=np.cos(a[...,2]),np.sin(a[...,2])
    return np.stack((a[...,0]+c*b[...,0]-s*b[...,1],
                     a[...,1]+s*b[...,0]+c*b[...,1],wrap(a[...,2]+b[...,2])),axis=-1)


def inverse(a):
    a=np.asarray(a,dtype=float);c,s=np.cos(a[...,2]),np.sin(a[...,2])
    return np.stack((-c*a[...,0]-s*a[...,1],s*a[...,0]-c*a[...,1],-a[...,2]),axis=-1)


def interpolate_pose(samples, times, max_gap=.1):
    """Nx4 [stamp,x,y,yaw]; nội suy góc đã unwrap, không ngoại suy hoặc bắc qua gap."""
    data=np.asarray(samples,dtype=float);times=np.asarray(times,dtype=float)
    if not np.isfinite(max_gap) or max_gap<=0:raise ValueError('max_gap must be finite and positive')
    if len(data)<2 or data.shape[1]!=4 or not np.isfinite(data).all():
        raise ValueError('Need finite timestamp/x/y/yaw samples')
    if np.any(np.diff(data[:,0])<0):raise ValueError('Pose timestamps moved backwards')
    # Giữ mẫu cuối nếu timestamp lặp; không giả là một khoảng thời gian mới.
    _,rev=np.unique(data[::-1,0],return_index=True);data=data[np.sort(len(data)-1-rev)]
    t=data[:,0];result=np.column_stack([np.interp(times,t,data[:,k] if k<3 else np.unwrap(data[:,3])) for k in (1,2,3)])
    result[:,2]=wrap(result[:,2]);i=np.searchsorted(t,times);left=np.maximum(i-1,0);right=np.minimum(i,len(t)-1)
    exact=(np.abs(times-t[right])<1e-9)|(np.abs(times-t[left])<1e-9)
    good=(times>=t[0])&(times<=t[-1])&(((t[right]-t[left])<=max_gap)|exact)
    result[~good]=np.nan
    return result


def aligned_errors(amcl, gt, map_to_world=None):
    """So sánh tại cùng timestamp GT; transform duy nhất cố định cho toàn lượt."""
    if map_to_world is None:raise ValueError('Missing explicit map→world registration')
    gt=np.asarray(gt,dtype=float);times=np.unique(gt[:,0])
    a=compose(map_to_world,interpolate_pose(amcl,times))
    g=interpolate_pose(gt,times)
    valid=np.isfinite(a).all(axis=1)&np.isfinite(g).all(axis=1)
    error=a-g;error[:,2]=wrap(error[:,2])
    return times[valid],a[valid],g[valid],error[valid]
