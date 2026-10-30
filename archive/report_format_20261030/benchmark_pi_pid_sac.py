#!/usr/bin/env python3
"""Một báo cáo offline cố định PI / PID / PI + SAC; không mở ROS hay train model."""
import argparse
from collections import defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import html
import json
import math
from pathlib import Path
import statistics
import zipfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
GROUPS = ('PI', 'PID', 'PI_SAC')
LABELS = {'PI':'PI', 'PID':'PID', 'PI_SAC':'PI + SAC'}
JERK_CUTOFF_HZ = 5.0
MAX_DEVIATION = 0.20
# key: nhóm, tên tiếng Việt, đơn vị, hướng tốt hơn. None: chỉ mô tả, không xếp hạng.
METRICS = {
 'mission_success_pct':('1. Hoàn thành','Đến đích theo đủ điều kiện AMCL và GT','%','max'),
 'nav_success_pct':('1. Hoàn thành','Nav2 thông báo SUCCEEDED','%','max'),
 'deviation_violation_pct':('1. Hoàn thành','Lượt vi phạm lệch đường AMCL hoặc GT','%','min'),
 'completion_time_s':('1. Hoàn thành','Thời gian hoàn thành, chỉ lượt đạt chuẩn','s','min'),
 'average_speed_m_s':('1. Hoàn thành','Vận tốc trung bình từ GT','m/s',None),
 'amcl_goal_xy_m':('1. Hoàn thành','Sai số vị trí AMCL cuối lượt','m','min'),
 'amcl_goal_yaw_rad':('1. Hoàn thành','Sai số hướng AMCL cuối lượt','rad','min'),
 'amcl_violation_pct':('1. Hoàn thành','Lượt AMCL vượt giới hạn lệch đường','%','min'),
 'gt_violation_pct':('1. Hoàn thành','Lượt GT vượt giới hạn lệch đường','%','min'),
 'gt_goal_xy_m':('1. Hoàn thành','Sai số vị trí GT cuối lượt','m','min'),
 'gt_goal_yaw_rad':('1. Hoàn thành','Sai số hướng GT tại đích','rad','min'),
 'amcl_rmse_m':('2. Bám đường','AMCL: RMSE ngang cân thời gian','m','min'),
 'amcl_p95_m':('2. Bám đường','AMCL: P95 |e_y| cân thời gian','m','min'),
 'amcl_max_m':('2. Bám đường','AMCL: Max |e_y|','m','min'),
 'amcl_heading_rmse_rad':('2. Bám đường','AMCL: RMSE lỗi hướng','rad','min'),
 'gt_rmse_m':('2. Bám đường','Ground truth: RMSE ngang cân thời gian','m','min'),
 'gt_p95_m':('2. Bám đường','Ground truth: P95 |e_y| cân thời gian','m','min'),
 'gt_max_m':('2. Bám đường','Ground truth: Max |e_y|','m','min'),
 'gt_heading_rmse_rad':('2. Bám đường','Ground truth: RMSE lỗi hướng','rad','min'),
 'az_rms_m_s2':('3. IMU','RMS a_z đã trừ trọng lực','m/s²','min'),
 'az_peak_m_s2':('3. IMU','Peak |a_z| đã trừ trọng lực','m/s²','min'),
 'roll_rate_rms_rad_s':('3. IMU','RMS vận tốc góc roll, body p','rad/s','min'),
 'pitch_rate_rms_rad_s':('3. IMU','RMS vận tốc góc pitch, body q','rad/s','min'),
 'jerk_rms_m_s3':('3. IMU','RMS jerk z, lọc 5 Hz','m/s³','min'),
 'jerk_p95_m_s3':('3. IMU','P95 |jerk z|, lọc 5 Hz','m/s³','min'),
 'wheel_rmse_rad_s':('5. Chấp hành','RMSE bám vận tốc hai bánh','rad/s','min'),
 'torque_rms_nm':('5. Chấp hành','RMS mô-men tổng, trung bình hai bánh','N·m','min'),
 'torque_peak_nm':('5. Chấp hành','Peak |mô-men tổng| trên hai bánh','N·m','min'),
 'sac_contribution_pct':('5. Chấp hành','Đóng góp |SAC| / (|PI| + |SAC|)','%',None),
 'slew_limit_pct':('5. Chấp hành','Chu kỳ chạm giới hạn slew trên hai bánh','%','min'),
 'saturation_pct':('5. Chấp hành','Chu kỳ bão hòa mô-men trên hai bánh','%','min'),
}
WINDOW_METRICS = {
 'local_peak_error_m':('Peak |e_y| GT','m'), 'local_rmse_m':('Local RMSE GT','m'),
 'peak_az_m_s2':('Peak |a_z|','m/s²'), 'peak_jerk_m_s3':('Peak |jerk z| lọc','m/s³'),
 'speed_drop_pct':('Sụt vận tốc so với trước gờ','%'), 'settling_time_s':('Ổn định sau tâm gờ','s'),
}


def wrap(value):
    return np.arctan2(np.sin(value), np.cos(value))


def weighted(values, weights):
    v,w = np.asarray(values,dtype=float),np.asarray(weights,dtype=float)
    good = np.isfinite(v)&np.isfinite(w)&(w>0)
    return float(np.average(v[good],weights=w[good])) if good.any() else None


def rms(values, weights):
    mean = weighted(np.asarray(values,dtype=float)**2,weights)
    return math.sqrt(mean) if mean is not None else None


def percentile(values, q=95, weights=None):
    v=np.asarray(values,dtype=float)
    if weights is None:
        v=v[np.isfinite(v)]
        return float(np.percentile(v,q)) if len(v) else None
    w=np.asarray(weights,dtype=float);good=np.isfinite(v)&np.isfinite(w)&(w>0)
    if not good.any():return None
    v,w=v[good],w[good];order=np.argsort(v);v,w=v[order],w[order]
    return float(v[min(np.searchsorted(np.cumsum(w),q/100*w.sum()),len(v)-1)])


def peak(values):
    v=np.asarray(values,dtype=float)
    return float(np.max(np.abs(v[np.isfinite(v)]))) if np.isfinite(v).any() else None


def stats(values):
    v=[float(x) for x in values if x is not None and math.isfinite(x)]
    return dict(n=len(v),mean=statistics.mean(v) if v else None,
                std=statistics.stdev(v) if len(v)>1 else None,
                median=float(np.median(v)) if v else None,p95=float(np.percentile(v,95)) if v else None)


def array(rows,key,width=None):
    result=[]
    for row in rows:
        value=row.get(key)
        if isinstance(value,str):
            try:value=json.loads(value)
            except json.JSONDecodeError:pass
        if width:
            if not isinstance(value,(list,tuple)) or len(value)!=width:value=[np.nan]*width
        else:value=np.nan if value is None else value
        result.append(value)
    return np.asarray(result,dtype=float)


def projection(x,y,yaw,points):
    """Cùng định nghĩa hình chiếu đoạn thẳng với environment, xử lý theo vector."""
    x,y,yaw=np.asarray(x),np.asarray(y),np.asarray(yaw)
    error=np.full(len(x),np.nan);heading=error.copy()
    best=np.full(len(x),np.inf);progress=np.full(len(x),np.nan);prefix=0.0
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length<1e-9:continue
        u=np.clip(((x-a[0])*dx+(y-a[1])*dy)/(length*length),0,1)
        px,py=a[0]+u*dx,a[1]+u*dy
        distance=np.hypot(x-px,y-py);take=distance<best
        error[take]=((x-px)*(-dy)+(y-py)*dx)[take]/length
        heading[take]=wrap(yaw-math.atan2(dy,dx))[take];best[take]=distance[take]
        progress[take]=prefix+u[take]*length;prefix+=length
    return error,heading,progress


def filtered_jerk(acceleration,times,cutoff=JERK_CUTOFF_HZ):
    """Đạo hàm a_z rồi LPF bậc một; cùng cutoff cho mọi nhóm, bỏ qua mất mẫu."""
    az=np.asarray(acceleration);t=np.asarray(times)
    jerk=np.full(len(t),np.nan);state=0.0;previous=None
    for i in range(len(t)):
        if not math.isfinite(az[i]) or not math.isfinite(t[i]):
            previous=None;state=0.0;continue
        if previous is not None:
            dt=t[i]-t[previous]
            if dt==0:continue
            if 0<dt<=0.10:
                alpha=math.exp(-2*math.pi*cutoff*dt)
                state=alpha*state+(1-alpha)*(az[i]-az[previous])/dt
                jerk[i]=state
            else:state=0.0
        previous=i
    return jerk


def obstacle_positions(world=None):
    path=Path(world) if world else ROOT/'src/nino_description/worlds/long_hall.sdf'
    tree=ET.parse(path)
    # Chỉ collision của cable_bumps; không đếm visual, tường hoặc đèn.
    model=tree.find(".//model[@name='cable_bumps']")
    if model is None:return []
    offset=float((model.findtext('pose') or '0 0 0 0 0 0').split()[0])
    return sorted(offset+float(p.findtext('pose').split()[0]) for p in model.findall('.//collision')
                  if p.get('name','').startswith('cable_'))


def window_metrics(t,x,error,az,jerk,speed,weights,centers,radius=0.5):
    results=[]
    for center in centers:
        inside=np.isfinite(x)&(np.abs(x-center)<=radius)
        adjacent=inside&np.r_[False,inside[:-1]]
        w=np.where(adjacent,weights,0)
        valid_x=x[np.isfinite(x)]
        full=bool(len(valid_x) and valid_x.min()<=center-radius and valid_x.max()>=center+radius)
        pre=(x>=center-radius)&(x<center-radius/2)&np.isfinite(speed)
        pre_speed=float(np.median(speed[pre])) if pre.any() else None
        drop=None
        if pre_speed is not None and pre_speed>0.03 and np.any(inside&np.isfinite(speed)):
            drop=100*max(0,1-float(np.nanmin(speed[inside]))/pre_speed)
        crossing=np.flatnonzero((x>=center)&np.isfinite(x))
        settle=None;reason='not_crossed'
        if len(crossing) and np.any(x<center) and pre_speed is not None and pre_speed>0.03:
            begin=int(crossing[0]);next_center=next((c for c in centers if c>center),math.inf)
            base=az[pre&np.isfinite(az)]
            band=max(0.5,1.2*float(np.percentile(np.abs(base),95))) if len(base) else 0.5
            stable_since=None;reason='censored_by_next_obstacle_or_episode_end'
            for i in range(begin,len(t)):
                if x[i]>=next_center or t[i]-t[begin]>5:break
                good=(np.isfinite(error[i]) and np.isfinite(az[i]) and np.isfinite(speed[i])
                      and abs(error[i])<=0.05 and abs(az[i])<=band and speed[i]>=0.95*pre_speed)
                if i>begin and t[i]-t[i-1]>0.10:stable_since=None
                stable_since=(i if stable_since is None else stable_since) if good else None
                if stable_since is not None and t[i]-t[stable_since]>=0.5:
                    settle=float(t[stable_since]-t[begin]);reason='observed';break
        results.append(dict(center_s_m=center,full_window=full,samples=int(inside.sum()),
            local_peak_error_m=peak(np.where(inside,error,np.nan)),local_rmse_m=rms(error,w),
            peak_az_m_s2=peak(np.where(inside,az,np.nan)),peak_jerk_m_s3=peak(np.where(inside,jerk,np.nan)),
            speed_drop_pct=drop,settling_time_s=settle,settling_status=reason))
    return results


def episode_metrics(rows, reference, centers=(), radius=0.5, map_to_world=(0,0,0)):
    if len(rows)<2:raise ValueError('Cần ít nhất hai mẫu telemetry')
    t=array(rows,'simulation_time');delta=np.diff(t,prepend=0)
    if not np.all(np.isfinite(t)) or np.any(delta<0) or t[-1]<=t[0]:
        raise ValueError('Timestamp không hợp lệ hoặc không tăng')
    if any(r.get('outcome') for r in rows[:-1]):raise ValueError('Terminal sample xuất hiện giữa episode')
    if any(delta[i]==0 and (i!=len(rows)-1 or not rows[i].get('outcome')) for i in range(1,len(t))):
        raise ValueError('Timestamp lặp ngoài terminal sample')
    weights=delta.copy();outcome=rows[-1].get('outcome') or 'INCOMPLETE'
    if outcome not in ('SUCCEEDED','FAILED','TIME_LIMIT','INCOMPLETE'):raise ValueError('Outcome không hợp lệ')
    amcl_x,amcl_y,amcl_yaw=(array(rows,k) for k in ('amcl_x','amcl_y','amcl_yaw'))
    gt_x,gt_y,gt_yaw=(array(rows,k) for k in ('ground_truth_x','ground_truth_y','ground_truth_yaw'))
    points=np.array([[p['x'],p['y']] for p in reference['poses']],dtype=float)
    frames={r.get('ground_truth_frame') for r in rows if r.get('ground_truth_frame')}
    registered=map_to_world is not None and frames=={'world'}
    ex,ey,theta=map_to_world if registered else (0.0,0.0,0.0);c,s=math.cos(theta),math.sin(theta)
    world_points=points@np.array([[c,s],[-s,c]])+np.array([ex,ey])
    amcl_error,amcl_heading,amcl_s=projection(amcl_x,amcl_y,amcl_yaw,points)
    # Log cũ chỉ có lỗi đã tính; giữ nguyên nguồn AMCL, không coi là ground truth.
    if not np.isfinite(amcl_error).any():amcl_error=array(rows,'cross_track')
    if not np.isfinite(amcl_heading).any():amcl_heading=wrap(array(rows,'heading_error'))
    gt_error,gt_heading,gt_s=projection(gt_x,gt_y,gt_yaw,world_points)
    if not registered:gt_error[:]=np.nan;gt_heading[:]=np.nan;gt_s[:]=np.nan
    frames={r.get('ground_truth_frame') for r in rows if r.get('ground_truth_frame')}
    if frames and frames!={'world'}:raise ValueError('GT frame phải là world; cần đăng ký transform trước')
    metrics={k:None for k in METRICS}
    for label,error,heading in [('amcl',amcl_error,amcl_heading),('gt',gt_error,gt_heading)]:
        metrics[label+'_rmse_m']=rms(error,weights)
        metrics[label+'_p95_m']=percentile(np.abs(error),weights=weights)
        metrics[label+'_max_m']=peak(error)
        metrics[label+'_heading_rmse_rad']=rms(wrap(heading),weights)
    acc=array(rows,'acceleration',3);gyro=array(rows,'gyro',3);az=acc[:,2]
    imu_t=array(rows,'imu_stamp_s')
    # Không dùng timestamp vòng lặp thay timestamp IMU bị thiếu.
    jerk=filtered_jerk(az,imu_t)
    metrics.update(az_rms_m_s2=rms(az,weights),az_peak_m_s2=peak(az),
                   roll_rate_rms_rad_s=rms(gyro[:,0],weights),pitch_rate_rms_rad_s=rms(gyro[:,1],weights),
                   jerk_rms_m_s3=rms(jerk,weights),jerk_p95_m_s3=percentile(np.abs(jerk),weights=weights))
    measured,target=array(rows,'wheel_velocity',2),array(rows,'wheel_target',2)
    metrics['wheel_rmse_rad_s']=rms(np.sqrt(np.mean((measured-target)**2,axis=1)),weights)
    total,base,residual=(array(rows,k,2) for k in ('total_torque','base_torque','residual_applied'))
    metrics['torque_rms_nm']=rms(np.sqrt(np.mean(total**2,axis=1)),weights)
    metrics['torque_peak_nm']=peak(total.flatten())
    numerator=np.sum(np.abs(residual),axis=1);denominator=np.sum(np.abs(base)+np.abs(residual),axis=1)
    ratio=np.divide(numerator,denominator,out=np.full(len(t),np.nan),where=denominator>1e-6)
    ratio[~np.all(np.isfinite(base)&np.isfinite(residual),axis=1)]=np.nan
    contribution=weighted(ratio,weights)
    metrics['sac_contribution_pct']=100*contribution if contribution is not None else None
    cycles=array(rows,'control_cycles')
    cycle_values=cycles[np.isfinite(cycles)]
    if np.any(np.diff(cycle_values)<0):raise ValueError('Controller cycle counter moved backwards')
    for key,out in [('slew_cycles','slew_limit_pct'),('saturation_cycles','saturation_pct')]:
        counts=array(rows,key,2)
        if np.all(np.isfinite([cycles[0],cycles[-1]])) and cycles[-1]>cycles[0] and np.all(np.isfinite(counts[[0,-1]])):
            difference=counts[-1]-counts[0]
            if np.any(difference<0):raise ValueError('Controller counter reset giữa episode')
            metrics[out]=100*float(np.sum(difference))/(2*(cycles[-1]-cycles[0]))
        else:
            flags=array(rows,'slew_limited' if key=='slew_cycles' else 'saturated',2)
            value=weighted(np.mean(flags,axis=1),weights)
            metrics[out]=100*value if value is not None else None
    speed=array(rows,'ground_truth_speed')
    good=np.isfinite(gt_x)&np.isfinite(gt_y)
    if np.all(good) and frames=={'world'}:metrics['average_speed_m_s']=float(np.hypot(np.diff(gt_x),np.diff(gt_y)).sum()/t[-1])
    goal=points[-1];world_goal=world_points[-1]
    amcl_xy=math.hypot(amcl_x[-1]-goal[0],amcl_y[-1]-goal[1])
    amcl_yaw_error=abs(float(wrap(amcl_yaw[-1]-reference['poses'][-1]['yaw'])))
    gt_xy=math.hypot(gt_x[-1]-world_goal[0],gt_y[-1]-world_goal[1])
    gt_yaw_error=abs(float(wrap(gt_yaw[-1]-reference['poses'][-1]['yaw']-theta)))
    metrics['amcl_goal_xy_m']=amcl_xy if math.isfinite(amcl_xy) else None
    metrics['amcl_goal_yaw_rad']=amcl_yaw_error if math.isfinite(amcl_yaw_error) else None
    for label,error in [('amcl',amcl_error),('gt',gt_error)]:
        maximum=peak(error)
        metrics[label+'_violation_pct']=100.0 if maximum is not None and maximum>MAX_DEVIATION else 0.0 if np.all(np.isfinite(error)) else None
    metrics['gt_goal_xy_m']=gt_xy if math.isfinite(gt_xy) else None
    metrics['gt_goal_yaw_rad']=gt_yaw_error if math.isfinite(gt_yaw_error) else None
    metrics['nav_success_pct']=100.0 if outcome=='SUCCEEDED' else 0.0
    if not registered:metrics['gt_goal_xy_m']=metrics['gt_goal_yaw_rad']=None
    errors_complete=all(np.all(np.isfinite(e)) for e in (amcl_error,gt_error))
    violation=any(peak(e) is not None and peak(e)>MAX_DEVIATION for e in (amcl_error,gt_error))
    metrics['deviation_violation_pct']=100.0 if violation else 0.0 if errors_complete else None
    checked=registered and errors_complete and all(math.isfinite(v) for v in (amcl_xy,amcl_yaw_error,gt_xy,gt_yaw_error))
    if checked:
        success=outcome=='SUCCEEDED' and not violation and amcl_xy<=0.25 and gt_xy<=0.25 and amcl_yaw_error<=0.25 and gt_yaw_error<=0.25
        metrics['mission_success_pct']=100.0 if success else 0.0
        if success:metrics['completion_time_s']=float(t[-1])
    hashes={r.get('reference_geometry_sha256') for r in rows if r.get('reference_geometry_sha256')}
    if len(hashes)>1:raise ValueError('Reference thay đổi trong episode')
    _,_,centers_s=projection(np.asarray(centers),np.zeros(len(centers)),np.zeros(len(centers)),world_points)
    if not registered:centers_s[:]=np.nan
    windows=window_metrics(t,gt_s,gt_error,az,jerk,speed,weights,centers_s,radius)
    for w,world_x in zip(windows,centers):w['center_world_x_m']=world_x
    trace=dict(reference_world_x=world_points[:,0].tolist() if registered else [],
               reference_world_y=world_points[:,1].tolist() if registered else [],
               amcl_world_x=(c*amcl_x-s*amcl_y+ex).tolist() if registered else [None]*len(t),
               amcl_world_y=(s*amcl_x+c*amcl_y+ey).tolist() if registered else [None]*len(t),
               t=t.tolist(),s=gt_s.tolist(),amcl_s=amcl_s.tolist(),x=gt_x.tolist() if registered else [None]*len(t),y=gt_y.tolist() if registered else [None]*len(t),amcl_x=amcl_x.tolist(),amcl_y=amcl_y.tolist(),
               error=gt_error.tolist(),amcl_error=amcl_error.tolist(),az=az.tolist(),jerk=jerk.tolist(),
               torque_left=total[:,0].tolist(),torque_right=total[:,1].tolist())
    return dict(outcome=outcome,reason=rows[-1].get('reason',''),metrics=metrics,windows=windows,
                duration_s=float(t[-1]),samples=len(rows),reference_hashes=sorted(hashes),trace=trace,
                schema={key:dict(present_samples=sum(key in row and row[key] is not None for row in rows),total_samples=len(rows))
                        for key in ('simulation_time','imu_stamp_s','amcl_x','amcl_y','amcl_yaw','ground_truth_x','ground_truth_y',
                                    'ground_truth_yaw','acceleration','gyro','wheel_target','wheel_velocity',
                                    'base_torque','residual_applied','total_torque','control_cycles','slew_cycles','saturation_cycles')},
                missing_reasons=missing_reasons(metrics,rows,registered),
                coverage={k:float(np.sum(weights[np.isfinite(v)])/np.sum(weights)) for k,v in
                          [('amcl',amcl_error),('ground_truth',gt_error),('az',az),('jerk',jerk)]})


def missing_reasons(metrics,rows,registered):
    audit=schema_audit(rows);result={}
    signals={
      'az_rms_m_s2':['acceleration'],'az_peak_m_s2':['acceleration'],
      'roll_rate_rms_rad_s':['gyro'],'pitch_rate_rms_rad_s':['gyro'],
      'jerk_rms_m_s3':['acceleration','imu_stamp_s'],'jerk_p95_m_s3':['acceleration','imu_stamp_s'],
      'wheel_rmse_rad_s':['wheel_target','wheel_velocity'],
      'torque_rms_nm':['total_torque'],'torque_peak_nm':['total_torque'],
      'sac_contribution_pct':['base_torque','residual_applied'],
      'average_speed_m_s':['ground_truth_x','ground_truth_y']}
    for key,value in metrics.items():
        if value is not None:continue
        if key.startswith('gt_') and not registered:
            reason='No verified map/world registration or GT world-frame label; GT is not AMCL'
        elif key=='completion_time_s':reason='No verified successful completion under all goal/deviation conditions'
        elif key=='sac_contribution_pct' and all(audit[k]['valid_samples'] for k in signals[key]):
            reason='No finite torque samples with |PI|+|SAC| > 1e-6 Nm; zero/near-zero denominators excluded'
        else:
            required=signals.get(key,[])
            if key.startswith('gt_') or key=='mission_success_pct':required+=['ground_truth_x','ground_truth_y','ground_truth_yaw']
            absent=[k for k in required if not audit[k]['valid_samples']]
            reason=('Missing/invalid signal: '+', '.join(absent)) if absent else 'Insufficient valid timed samples; inspect schema, timestamps, registration and terminal outcome'
        result[key]=reason
    return result


def read_rows(path):
    if path.suffix=='.csv':
        with path.open(newline='') as stream:return list(csv.DictReader(stream))
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def read_json(path,default=None):
    return json.loads(path.read_text()) if path.exists() else default


def reference_for(folder,config):
    candidates=[folder.parent/'reference_path.json']
    path=config.get('environment',{}).get('reference_path_file')
    if path:candidates.append(Path(path))
    for path in candidates:
        if path.is_file():return read_json(path)
    raise ValueError(f'Thiếu reference_path.json cho {folder}; không tự đoán đường tham chiếu')


def checkpoint_evidence(metadata):
    value=metadata.get('model')
    if not value:return dict(verified=False,reason='No checkpoint path in metadata')
    path=Path(value)
    if not path.is_file():return dict(verified=False,path=str(path),reason='Checkpoint no longer available; timestep not inferred from folder name')
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    expected=metadata.get('checkpoint_evidence') or {}
    if expected.get('sha256') and expected['sha256']!=digest:raise ValueError('SAC checkpoint hash differs from evaluation metadata')
    with zipfile.ZipFile(path) as archive:
        data=json.loads(archive.read('data'))
        if 'ent_coef' not in data:raise ValueError('Checkpoint is not an SB3 SAC archive')
        timesteps=data['num_timesteps']
    if isinstance(timesteps,bool) or not isinstance(timesteps,int) or timesteps<0:raise ValueError('Invalid checkpoint timestep')
    if expected.get('timesteps') is not None and expected['timesteps']!=timesteps:raise ValueError('Checkpoint timestep mismatch')
    source=path.parent.parent if path.parent.name=='checkpoints' else path.parent
    for filename,key in [('config.yaml','config_sha256'),('metadata.json','metadata_sha256')]:
        if expected.get(key):
            candidate=source/filename
            if not candidate.is_file():return dict(verified=False,path=str(path),timesteps=timesteps,sha256=digest,reason='Source '+filename+' unavailable')
            if hashlib.sha256(candidate.read_bytes()).hexdigest()!=expected[key]:raise ValueError('SAC source '+filename+' changed since evaluation')
    return dict(verified=bool(expected.get('sha256') and expected.get('config_sha256') and expected.get('metadata_sha256')),path=str(path),timesteps=timesteps,sha256=digest,
                reason=None if expected.get('sha256') and expected.get('config_sha256') and expected.get('metadata_sha256') else 'Archive read; historical config/metadata/checkpoint hash proof is incomplete')


def schema_audit(rows):
    sizes={'acceleration':3,'gyro':3,'wheel_target':2,'wheel_velocity':2,'total_torque':2,
           'base_torque':2,'residual_applied':2,'slew_cycles':2,'saturation_cycles':2}
    fields=['simulation_time','imu_stamp_s','amcl_x','amcl_y','amcl_yaw','ground_truth_x','ground_truth_y',
            'ground_truth_yaw','acceleration','gyro','wheel_target','wheel_velocity','base_torque',
            'residual_applied','total_torque','control_cycles','slew_cycles','saturation_cycles']
    audit={}
    for key in fields:
        valid=0;present=0
        for row in rows:
            value=row.get(key)
            if value is None:continue
            present+=1
            try:
                if isinstance(value,str):value=json.loads(value)
                if key in sizes and (not isinstance(value,(list,tuple)) or len(value)!=sizes[key]):continue
                if not np.all(np.isfinite(np.asarray(value,dtype=float))):continue
                valid+=1
            except (ValueError,TypeError,json.JSONDecodeError):pass
        audit[key]=dict(present_samples=present,valid_samples=valid,total_samples=len(rows))
    return audit


def load_group(folder,label,centers,radius):
    folder=Path(folder).expanduser().resolve()
    if not folder.is_dir():raise ValueError(f'Không tìm thấy thư mục {folder}')
    metadata=read_json(folder/'metadata.json',{})
    config=yaml.safe_load((folder/'config.yaml').read_text()) if (folder/'config.yaml').exists() else {}
    proof=checkpoint_evidence(metadata) if label=='PI_SAC' else None
    for filename,key in [('config.yaml','config_snapshot_sha256'),('controller.yaml','controller_snapshot_sha256')]:
        if metadata.get(key) and hashlib.sha256((folder/filename).read_bytes()).hexdigest()!=metadata[key]:
            raise ValueError('Snapshot changed since evaluation: '+str(folder/filename))
    nav=config.get('environment',{}).get('nav2_params_file')
    if metadata.get('nav2_snapshot_sha256') and nav and Path(nav).is_file():
        if hashlib.sha256(Path(nav).read_bytes()).hexdigest()!=metadata['nav2_snapshot_sha256']:
            raise ValueError('Nav2 snapshot changed since evaluation')
    controller_file=folder/'controller.yaml'
    controller=yaml.safe_load(controller_file.read_text())['effort_drive']['ros__parameters'] if controller_file.exists() else metadata.get('controller',{})
    if metadata.get('controller') and controller!=metadata['controller']:raise ValueError('Controller snapshot differs from metadata')
    if label in ('PI','PI_SAC') and controller:
        if any(controller.get(k)!=v for k,v in [('velocity_kp',0.30),('velocity_ki',0.10),('velocity_kd',0.0)]):
            raise ValueError('Original PI gains changed in '+label)
    reference=reference_for(folder,config)
    expected={'PI':'PI','PID':'PID','PI_SAC':'SAC'}[label]
    if label=='PI_SAC' and metadata.get('mode') in ('baseline','train','smoke'):raise ValueError('SAC-dir must be deterministic evaluation, not baseline/train')
    if label!='PI_SAC' and metadata.get('mode')=='evaluate':raise ValueError('Baseline directory claims SAC evaluation mode')
    if label=='PID' and controller and any(controller.get(k,0)<=0 for k in ('velocity_kp','velocity_ki','velocity_kd')):
        raise ValueError('PID directory does not contain a positive Kp/Ki/Kd profile')
    if metadata.get('controller_mode') and metadata['controller_mode']!=expected:
        raise ValueError(f'{label}: metadata controller_mode không khớp')
    if label=='PI_SAC' and metadata.get('deterministic') is False:raise ValueError('SAC evaluation phải deterministic')
    episodes,audit=[],[]
    for directory in sorted(p for p in folder.glob('episode_*') if p.is_dir()):
        path=next((directory/name for name in ('telemetry.jsonl','steps.jsonl','telemetry.csv')
                   if (directory/name).is_file()),None)
        if path is None or not path.stat().st_size:
            audit.append(dict(kind='startup_without_samples',source=str(directory)));continue
        try:
            rows=read_rows(path)
            schema=schema_audit(rows)
            record=episode_metrics(rows,reference,centers,radius,metadata.get('map_to_world'))
            record['schema']=schema
            if record['reference_hashes'] and metadata.get('reference_geometry_sha256') and record['reference_hashes']!=[metadata['reference_geometry_sha256']]:
                raise ValueError('Telemetry reference hash differs from run metadata')
            if label!='PI_SAC' and np.any(np.nan_to_num(array(rows,'residual_applied',2))!=0):
                raise ValueError('Baseline có residual SAC khác 0')
            if not metadata.get('map_to_world'):audit.append(dict(kind='missing_frame_registration',source=str(path),reason='GT-vs-reference metrics are N/A without map_to_world'))
            record.update(episode=directory.name,source=str(path),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            episodes.append(record)
        except (ValueError,KeyError,TypeError,json.JSONDecodeError) as exc:
            audit.append(dict(kind='invalid_episode',source=str(path),reason=str(exc)))
    for name in ('startup_recovery.jsonl','attempt_events.jsonl'):
        if (folder/name).exists():
            audit.extend(dict(kind=name,detail=json.loads(line)) for line in (folder/name).read_text().splitlines() if line.strip())
    for name in ('infrastructure_fault.json','exception.json'):
        if (folder/name).exists():audit.append(dict(kind=name,detail=read_json(folder/name)))
    complete=[e for e in episodes if e['outcome']!='INCOMPLETE']
    return dict(label=label,path=str(folder),metadata=metadata,config=config,reference=reference,
                checkpoint_proof=proof,controller=controller,
                episodes=episodes,audit=audit,complete_count=len(complete),
                summary={k:stats(e['metrics'][k] for e in complete) for k in METRICS})


def clean(value):
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    if isinstance(value,(np.floating,float)) and not math.isfinite(value):return None
    return value


def write_csv(path,rows):
    with path.open('w',newline='') as stream:
        fields=list(dict.fromkeys(key for row in rows for key in row))
        if not fields:return
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(clean(rows))


def format_stats(value):
    if value['n']==0:return 'N/A'
    sd=f"{value['std']:.4g}" if value['std'] is not None else 'N/A'
    return f"{value['mean']:.4g} ± {sd}"


def latex_escape(text):
    chars={'\\':r'\textbackslash{}','_':r'\_','%':r'\%','&':r'\&','#':r'\#','{':r'\{','}':r'\}'}
    return ''.join(chars.get(c,c) for c in text)


def make_html(groups,centers,output,notices):
    palette=['#2463a4','#d67b17','#14866b']
    rows=[]
    for key,(section,name,unit,direction) in METRICS.items():
        values=[g['summary'][key] for g in groups];valid=[v['mean'] for v in values if v['mean'] is not None]
        best=(min(valid) if direction=='min' else max(valid)) if valid and direction and not any(n.startswith('CONFOUND') for n in notices) else None
        cells=[]
        for v in values:
            text=html.escape(format_stats(v))
            if best is not None and v['mean'] is not None and math.isclose(v['mean'],best,rel_tol=1e-9,abs_tol=1e-12):text='<strong>'+text+'</strong>'
            more=f"Median {v['median']:.4g} · P95 {v['p95']:.4g} · n={v['n']}" if v['n'] else 'Không đủ dữ liệu'
            cells.append('<td>'+text+'<small>'+html.escape(more)+'</small></td>')
        rows.append('<tr><th>'+html.escape(section+' · '+name+' ('+unit+')')+'</th>'+''.join(cells)+'</tr>')
    windows=[]
    for center in centers:
        for key,(name,unit) in WINDOW_METRICS.items():
            values=[stats(w[key] for e in g['episodes'] if e['outcome']!='INCOMPLETE'
                          for w in e['windows'] if w['center_world_x_m']==center and w['full_window']) for g in groups]
            valid=[v['mean'] for v in values if v['mean'] is not None];best=min(valid) if valid and not any(n.startswith('CONFOUND') for n in notices) else None
            cells=[]
            for v in values:
                text=html.escape(format_stats(v))
                if best is not None and v['mean'] is not None and math.isclose(v['mean'],best):text='<strong>'+text+'</strong>'
                more=f"Median {v['median']:.4g} · P95 {v['p95']:.4g} · n={v['n']}" if v['n'] else 'Chưa đủ cửa sổ đầy đủ hoặc bị kiểm duyệt'
                cells.append('<td>'+text+'<small>'+html.escape(more)+'</small></td>')
            windows.append('<tr><th>'+html.escape(f'Gờ x={center:g} m · {name} ({unit})')+'</th>'+''.join(cells)+'</tr>')
    episodes=''.join('<tr><td>'+LABELS[g['label']]+'</td><td>'+html.escape(e['episode'])+'</td><td>'+e['outcome']+'</td><td>'+html.escape(str(e['metrics']['mission_success_pct']))+'</td><td>'+html.escape(e['reason'])+'</td></tr>' for g in groups for e in g['episodes'])
    data=[dict(label=LABELS[g['label']],color=palette[i],traces=[e['trace'] for e in g['episodes'] if e['outcome']!='INCOMPLETE'],
               reference=g['reference']) for i,g in enumerate(groups)]
    payload=json.dumps(clean(dict(groups=data,centers=([w['center_s_m'] for w in groups[0]['episodes'][0]['windows']] if groups[0]['episodes'] else []))),ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    audit=html.escape(json.dumps(clean({g['label']:dict(events=g['audit'],checkpoint=g['checkpoint_proof'],
        gains=g['metadata'].get('gains'),pid_calibration=g['metadata'].get('pid_calibration'),schema_by_episode=[dict(episode=e['episode'],schema=e['schema'],
        missing_reasons=e['missing_reasons'],coverage=e['coverage']) for e in g['episodes']]) for g in groups}),ensure_ascii=False,indent=2))
    header='<tr><th>Chỉ số</th><th>PI</th><th>PID</th><th>PI + SAC</th></tr>'
    page='''<!doctype html><html lang="vi"><meta charset="utf-8"><title>PI / PID / PI + SAC</title>
<style>body{font:15px system-ui;margin:24px auto;max-width:1280px;padding:0 18px;color:#213047;background:#f7f9fc}h1{font-size:28px}table{border-collapse:collapse;width:100%;background:white;margin:16px 0}td,th{border-bottom:1px solid #dce3ec;padding:9px;text-align:left}th{font-size:13px}small{display:block;color:#677489;font-size:11px;margin-top:4px}strong{color:#006c4c}section{background:white;padding:15px;margin:16px 0;border-radius:8px}svg{width:100%;height:auto}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}.warning{background:#fff0c5;padding:12px}.scroll{overflow:auto}select{padding:6px}</style>
<h1>Đối sánh PI / PID / PI + SAC</h1><p>Mean ± sample SD; Median và P95 qua các episode ở dòng nhỏ. n là số lượt có dữ liệu cho từng metric. Lượt FAILED/TIME_LIMIT vẫn giữ; thời gian hoàn thành chỉ xét lượt đạt chuẩn.</p>
<div class="warning">NOTICES</div><section><label>Chọn lượt tương ứng theo thứ tự log: <select id="episode"></select></label><p>Màu: <span style="color:#2463a4">PI</span> · <span style="color:#d67b17">PID</span> · <span style="color:#14866b">PI + SAC</span>. Đường liền GT, nét đứt AMCL. Cùng nhãn seed không bảo đảm cùng nhiễu Gazebo/AMCL.</p>
<h3>Quỹ đạo 2D (m)</h3><svg id="trajectory" viewBox="0 0 1000 280"></svg>
<h3>Lỗi ngang e_y theo tiến độ dọc đường s (m)</h3><svg id="error" viewBox="0 0 1000 280"></svg>
<h3>a_z đã trừ trọng lực theo s (m/s²)</h3><svg id="az" viewBox="0 0 1000 280"></svg>
<h3>Jerk z lọc 5 Hz theo s (m/s³)</h3><svg id="jerk" viewBox="0 0 1000 280"></svg>
<h3>Mô-men tổng trái/phải theo s (N·m)</h3><svg id="torque" viewBox="0 0 1000 280"></svg></section>
<div class="scroll"><table>HEADERROWS</table></div><h2>4. Cửa sổ ±0.5 m dọc đường quanh tâm gờ</h2><p>Chỉ cửa sổ đầy đủ được tổng hợp tại đây. CSV giữ cả cửa sổ thiếu và trạng thái settling bị kiểm duyệt. Vạch gờ là tâm hình học, không phải cảm biến tiếp xúc bánh.</p><div class="scroll"><table>HEADERWINDOWS</table></div>
<h2>Từng lượt và lỗi hạ tầng</h2><table><tr><th>Nhóm</th><th>Episode</th><th>Nav outcome</th><th>Đạt đủ chuẩn (%)</th><th>Lý do</th></tr>EPISODES</table><details><summary>Audit, checkpoint/config, schema và lý do N/A</summary><pre>AUDIT</pre></details>
<section><h2>Phương pháp</h2><p>METHODS</p><p>Giá trị in đậm là mean nhỏ nhất (thời gian/lỗi/rung/torque/giới hạn), hoặc lớn nhất (thành công). Vận tốc và đóng góp SAC chỉ mô tả, không có hướng tối ưu phổ quát. Không suy luận ý nghĩa thống kê từ thứ hạng.</p><p>HTML này nhúng toàn bộ dữ liệu/biểu đồ, không dùng CDN hay Internet.</p></section>
<script id="data" type="application/json">PAYLOAD</script><script>
const D=JSON.parse(document.getElementById('data').textContent),NS='http://www.w3.org/2000/svg';
function el(tag,attr,text){let e=document.createElementNS(NS,tag);for(let[k,v]of Object.entries(attr||{}))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;return e}
function draw(id,series,marks,equal){let svg=document.getElementById(id);svg.replaceChildren();let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;for(let s of series)for(let p of s.points)if(Number.isFinite(p[0])&&Number.isFinite(p[1])){xmin=Math.min(xmin,p[0]);xmax=Math.max(xmax,p[0]);ymin=Math.min(ymin,p[1]);ymax=Math.max(ymax,p[1])}if(!Number.isFinite(xmin)){svg.append(el('text',{x:50,y:100},'N/A: thiếu tín hiệu'));return}if(xmax===xmin)xmax=xmin+1;if(ymax===ymin){ymin-=.1;ymax+=.1}let pad=(ymax-ymin)*.1;ymin-=pad;ymax+=pad;if(equal){let cy=(ymin+ymax)/2,span=Math.max(ymax-ymin,(xmax-xmin)*210/890);ymin=cy-span/2;ymax=cy+span/2}let X=x=>65+(x-xmin)/(xmax-xmin)*890,Y=y=>240-(y-ymin)/(ymax-ymin)*210;for(let i=0;i<=5;i++){let x=xmin+(xmax-xmin)*i/5,y=ymin+(ymax-ymin)*i/5;svg.append(el('line',{x1:65,y1:Y(y),x2:955,y2:Y(y),stroke:'#e0e6ef'}));svg.append(el('text',{x:10,y:Y(y)+4,'font-size':11},y.toFixed(3)));svg.append(el('text',{x:X(x),y:262,'font-size':11},x.toFixed(2)))}if(marks)for(let x of D.centers)if(Number.isFinite(x)&&x>=xmin&&x<=xmax)svg.append(el('line',{x1:X(x),x2:X(x),y1:30,y2:240,stroke:'#bbb','stroke-dasharray':'3 4'}));for(let s of series){let points=[];function flush(){if(points.length>1)svg.append(el('polyline',{points:points.join(' '),fill:'none',stroke:s.color,'stroke-width':1.4,'stroke-dasharray':s.dash?'5 4':''}));points=[]}for(let p of s.points){if(Number.isFinite(p[0])&&Number.isFinite(p[1]))points.push(X(p[0])+','+Y(p[1]));else flush()}flush()}}
function render(){let index=Number(document.getElementById('episode').value),plots={trajectory:[],error:[],az:[],jerk:[],torque:[]};for(let g of D.groups){let tr=g.traces[index];if(!tr)continue;function add(plot,x,y,dash=false){plots[plot].push({color:g.color,dash,points:x.map((v,i)=>[v,y[i]])})}add('trajectory',tr.x,tr.y);add('trajectory',tr.amcl_world_x,tr.amcl_world_y,true);add('error',tr.s,tr.error);add('error',tr.amcl_s,tr.amcl_error,true);add('az',tr.s,tr.az);add('jerk',tr.s,tr.jerk);add('torque',tr.s,tr.torque_left);add('torque',tr.s,tr.torque_right,true)}let guide=D.groups.map(g=>g.traces[index]).find(tr=>tr&&tr.reference_world_x?.length);if(guide)plots.trajectory.unshift({color:'#222',dash:true,points:guide.reference_world_x.map((x,i)=>[x,guide.reference_world_y[i]])});for(let[id,series]of Object.entries(plots))draw(id,series,id!=='trajectory',id==='trajectory')}
let menu=document.getElementById('episode'),count=Math.max(0,...D.groups.map(g=>g.traces.length));for(let i=0;i<count;i++){let o=document.createElement('option');o.value=i;o.textContent='Lượt '+(i+1);menu.append(o)}menu.onchange=render;render();
</script></html>'''
    methods=('Thành công chính: Nav2 SUCCEEDED, cả AMCL và GT có xy≤0.25 m, yaw≤0.25 rad và max lệch ngang≤0.20 m. '
      'GT world và reference map được đăng ký bằng map_to_world trong metadata (runner hiện spawn/initialpose cùng gốc, identity), không fit vào quỹ đạo thực. '
      'RMSE và P95 trong episode cân theo dt; thống kê Mean/SD/Median/P95 sau đó cân đều mỗi episode. '
      'a_z là trục Z thân xe đã trừ vector trọng lực theo quaternion IMU; gyro p/q là vận tốc góc body. '
      'Jerk là sai phân a_z trên timestamp IMU rồi lọc nhân quả bậc một 5 Hz; gap>0.1 s reset, mẫu lặp bỏ qua. '
      'Tỷ lệ limiter dùng chênh counter ở vòng điều khiển, nếu thiếu dùng cờ lấy mẫu và ghi trong metadata nguồn. '
      'Settling: từ lần tâm robot vượt tâm gờ, tìm |e_y|≤0.05 m, |a_z|≤max(0.5,1.2×P95 trước gờ), tốc độ≥95% median trước gờ liên tục 0.5 s. '
      'Dừng quan sát khi gặp gờ kế, hết episode hoặc sau 5 s; không ổn định được ghi N/A/censored, không ghi 0. '
      'Cửa sổ các gờ gần nhau có thể chồng lấn; không phải các quan sát độc lập. '
      'Mô-men là lệnh cuối sau limiter, không phải cảm biến lực; tỷ lệ SAC mô tả residual trước limiter cuối.')
    replacements={'NOTICES':'<br>'.join(html.escape(n) for n in notices),'HEADERROWS':header+''.join(rows),
        'HEADERWINDOWS':header+''.join(windows),'EPISODES':episodes,'AUDIT':audit,'METHODS':methods,'PAYLOAD':payload}
    for key,value in replacements.items():page=page.replace(key,value)
    output.write_text(page)


def build_report(pi_dir,pid_dir,sac_dir,output,centers=None):
    centers=obstacle_positions() if centers is None else sorted(centers)
    groups=[load_group(folder,label,centers,0.5) for label,folder in zip(GROUPS,(pi_dir,pid_dir,sac_dir))]
    notices=[]
    if len({str(Path(p).resolve()) for p in (pi_dir,pid_dir,sac_dir)})!=3:raise ValueError('Three distinct input directories are required')
    signatures=[]
    for g in groups:
        env=dict(g['config'].get('environment',{}))
        for key in ('effort_drive_config','benchmark_controller','resource_guard'):env.pop(key,None)
        nav=env.pop('nav2_params_file',None);env.pop('reference_path_file',None)
        nav_hash=hashlib.sha256(Path(nav).read_bytes()).hexdigest() if nav and Path(nav).is_file() else None
        limits={k:v for k,v in g['controller'].items() if k not in ('velocity_kp','velocity_ki','velocity_kd','derivative_filter_seconds')}
        signatures.append(json.dumps([env,limits,nav_hash,g['metadata'].get('map'),g['metadata'].get('map_to_world'),g['reference']],sort_keys=True))
        if not nav_hash or not limits:notices.append('CONFOUND: '+g['label']+' lacks Nav2/actuator evidence; identical conditions cannot be verified.')
        if g['label']=='PI_SAC' and not g['checkpoint_proof']['verified']:notices.append('SAC checkpoint/config not fully verified: '+str(g['checkpoint_proof'].get('reason')))
    if len(set(signatures))>1:notices.append('CONFOUND: environment/Nav2/frame/actuator/reference conditions differ; best-value highlighting is disabled.')
    reference_hashes={tuple(e['reference_hashes']) for g in groups for e in g['episodes'] if e['reference_hashes']}
    if len(reference_hashes)>1:notices.append('CONFOUND: các lượt không dùng cùng hình học tham chiếu; không diễn giải như so sánh cùng điều kiện.')
    for g in groups:
        if g['metadata'].get('experiment_purpose')=='smoke':notices.append(LABELS[g['label']]+': smoke ngắn, không phải đánh giá chất lượng.')
        if g['label']=='PID' and not (g['metadata'].get('pid_calibration') or {}).get('selected_profile_matches'):
            notices.append('PID profile chưa được xác nhận đạt tiêu chí hiệu chỉnh; không gọi là PID tối ưu.')
        if any(e['coverage']['ground_truth']<1 for e in g['episodes']):notices.append(LABELS[g['label']]+': ground truth thiếu mẫu; xem coverage, không suy ra từ AMCL.')
        if not g['complete_count']:notices.append(LABELS[g['label']]+': chưa có episode hoàn tất.')
    notices.append('PID dùng profile đã ghi trong metadata; kiểm tra hồ sơ hiệu chỉnh. Báo cáo không chứng minh tối ưu toàn cục hay tự động tốt hơn PI.')
    output=Path(output).expanduser().resolve();output.mkdir(parents=True,exist_ok=False)
    rows=[];window_rows=[];episode_rows=[]
    for g in groups:
        for key,value in g['summary'].items():
            section,name,unit,direction=METRICS[key]
            rows.append(dict(group=LABELS[g['label']],section=section,metric=key,description=name,unit=unit,**value))
        for e in g['episodes']:
            episode_rows.append(dict(group=LABELS[g['label']],episode=e['episode'],outcome=e['outcome'],
                source=e['source'],duration_s=e['duration_s'],**e['metrics'],**{'coverage_'+k:v for k,v in e['coverage'].items()}))
            window_rows.extend(dict(group=LABELS[g['label']],episode=e['episode'],**w) for w in e['windows'])
    write_csv(output/'summary.csv',rows);write_csv(output/'episodes.csv',episode_rows);write_csv(output/'obstacle_windows.csv',window_rows)
    window_summary=[]
    for g in groups:
        for center in centers:
            for key in WINDOW_METRICS:
                value=stats(w[key] for e in g['episodes'] if e['outcome']!='INCOMPLETE' for w in e['windows']
                            if w['full_window'] and w['center_world_x_m']==center)
                window_summary.append(dict(group=LABELS[g['label']],center_world_x_m=center,metric=key,**value))
    write_csv(output/'obstacle_summary.csv',window_summary)
    # Mean/SD, Median, P95 cho cả ba controller; booktabs dùng được với IEEEtran.
    def table_latex(entries,caption,label):
        lines=[r'\begin{table*}[t]',r'\centering',r'\scriptsize',
            r'\caption{'+caption+'}',r'\label{'+label+'}',
            r'\begin{tabular}{p{0.30\textwidth}*{9}{r}}',r'\toprule',
            r' & \multicolumn{3}{c}{PI} & \multicolumn{3}{c}{PID} & \multicolumn{3}{c}{PI+SAC} \\',
            r'\cmidrule(lr){2-4}\cmidrule(lr){5-7}\cmidrule(lr){8-10}',
            r'Metric & Mean $\pm$ SD & Median & P95 & Mean $\pm$ SD & Median & P95 & Mean $\pm$ SD & Median & P95 \\',r'\midrule']
        for key,unit,direction,values in entries:
            valid=[v['mean'] for v in values if v['mean'] is not None]
            best=(min(valid) if direction=='min' else max(valid)) if direction and valid and not any(n.startswith('CONFOUND') for n in notices) else None
            cells=[]
            for v in values:
                text=(f"{v['mean']:.4g} $\\pm$ "+(f"{v['std']:.4g}" if v['std'] is not None else '--')) if v['n'] else '--'
                if best is not None and v['mean'] is not None and math.isclose(best,v['mean'],rel_tol=1e-9,abs_tol=1e-12):text=r'\textbf{'+text+'}'
                cells.extend([text]+[f"{v[k]:.4g}" if v[k] is not None else '--' for k in ('median','p95')])
            text=latex_escape(key.replace('_',' '))
            unit=latex_escape(unit.replace('²','2').replace('³','3').replace('·',' '))
            lines.append(text+' ('+unit+') & '+' & '.join(cells)+r' \\')
        return lines+[r'\bottomrule',r'\end{tabular}',r'\end{table*}']
    tex=[r'% Requires \usepackage{booktabs}; IEEEtran-compatible. -- means unavailable, never zero imputation.',
         r'% Percentiles in these columns describe episode-level metric distributions; see CSV for sample counts.']
    entries=[(key,definition[2],definition[3],[g['summary'][key] for g in groups]) for key,definition in METRICS.items()]
    tex+=table_latex(entries,'PI, PID and PI+SAC: all completed outcomes retained.','tab:pi-pid-sac')
    for index,center in enumerate(centers):
        entries=[]
        for key,(_,unit) in WINDOW_METRICS.items():
            values=[stats(w[key] for e in g['episodes'] if e['outcome']!='INCOMPLETE'
                       for w in e['windows'] if w['center_world_x_m']==center and w['full_window']) for g in groups]
            entries.append((key,unit,'min',values))
            for g,value in zip(groups,values):
                rows.append(dict(group=LABELS[g['label']],section='4. Local obstacle',metric=f'obstacle_{center:g}_{key}',
                    description=WINDOW_METRICS[key][0],unit=unit,center_world_x_m=center,**value))
        tex+=table_latex(entries,f'Obstacle at world x={center:g} m; full local windows only.',f'tab:obstacle-{index+1}')
    write_csv(output/'summary.csv',rows)
    (output/'comparison_table.tex').write_text('\n'.join(tex)+'\n')
    report=dict(created_utc=datetime.now(timezone.utc).isoformat(),analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),groups=groups,obstacles_world_x_m=centers,
                jerk_cutoff_hz=JERK_CUTOFF_HZ,notices=notices,
                definitions={k:dict(section=v[0],description=v[1],unit=v[2],direction=v[3]) for k,v in METRICS.items()})
    (output/'report_data.json').write_text(json.dumps(clean(report),ensure_ascii=False,indent=2,allow_nan=False))
    (output/'audit.json').write_text(json.dumps({g['label']:g['audit'] for g in groups},ensure_ascii=False,indent=2))
    make_html(groups,centers,output/'report_PI_PID_SAC.html',notices)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pi-dir',required=True,type=Path)
    parser.add_argument('--pid-dir',required=True,type=Path)
    parser.add_argument('--sac-dir',required=True,type=Path)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--obstacle-x',nargs='+',type=float,help='Tọa độ world X; mặc định đọc cable_bumps trong SDF')
    args=parser.parse_args(argv)
    if args.obstacle_x and not all(math.isfinite(x) for x in args.obstacle_x):parser.error('Tọa độ gờ phải hữu hạn')
    try:build_report(args.pi_dir,args.pid_dir,args.sac_dir,args.output_dir,args.obstacle_x)
    except (ValueError,FileExistsError,FileNotFoundError,KeyError) as exc:parser.error(str(exc))
    print('Report:',args.output_dir.resolve()/'report_PI_PID_SAC.html')


if __name__=='__main__':main()
