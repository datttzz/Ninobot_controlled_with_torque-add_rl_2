#!/usr/bin/env python3
"""Phân tích log/rosbag định vị offline; không khởi chạy robot hay chỉnh tham số."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'rl_training'))
from plotting_compat import prepare_matplotlib_namespace
prepare_matplotlib_namespace()
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from localization_math import aligned_errors,compose,inverse,interpolate_pose,wrap
sys.path.insert(0,str(ROOT))
from rl_training.frame_registration import load_registration
from benchmark_pi_pid_sac import obstacle_positions,read_rows


def stamp(msg):return msg.sec+msg.nanosec*1e-9


def yaw(q):return np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def pose(p):return [p.position.x,p.position.y,float(yaw(p.orientation))]


def save_json(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False))


def write_csv(path,rows):
    with path.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def extract_bag(path):
    # Chỉ import ROS khi đọc bag; phần toán/plot không cần ROS.
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    db=sqlite3.connect(next(path.glob('*.db3')))
    topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
    result=dict(odom=[],gt=[],amcl=[],tf_map_odom=[],tf_odom_base=[],scans=[],gt3d=[],
                covariance=[],states=[],static_tf=[],counts={},initial_pose=[],particles=[],wheel=[],delivery={},tf_available=[])
    clock=None
    for tid,receipt,data in db.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
        name,kind=topics[tid];result['counts'][name]=result['counts'].get(name,0)+1
        if name not in ('/clock','/joint_states','/odom','/ground_truth/odom','/amcl_pose','/tf','/tf_static','/scan','/episode/state','/initialpose','/particle_cloud'):continue
        m=deserialize_message(data,kind)
        if name=='/clock':clock=stamp(m.clock);continue
        if hasattr(m,'header') and clock is not None:
            result['delivery'].setdefault(name,[]).append(clock-stamp(m.header.stamp))
        if name=='/joint_states':
            values=dict(zip(m.name,m.velocity))
            result['wheel'].append([stamp(m.header.stamp),values['left_wheel_joint'],values['right_wheel_joint']])
        elif name in ('/odom','/ground_truth/odom','/amcl_pose'):
            key={'/odom':'odom','/ground_truth/odom':'gt','/amcl_pose':'amcl'}[name]
            result[key].append([stamp(m.header.stamp),*pose(m.pose.pose)])
            if key=='gt':
                p,q=m.pose.pose.position,m.pose.pose.orientation
                result['gt3d'].append([stamp(m.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
            if key=='amcl':result['covariance'].append([stamp(m.header.stamp),m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]])
        elif name in ('/tf','/tf_static'):
            for tr in m.transforms:
                p=tr.transform.translation;q=tr.transform.rotation
                row=[stamp(tr.header.stamp),p.x,p.y,float(yaw(q))]
                if (tr.header.frame_id,tr.child_frame_id)==('map','odom'):
                    result['tf_map_odom'].append(row)
                    if clock is not None:
                        result['delivery'].setdefault('map_odom_future_header',[]).append(stamp(tr.header.stamp)-clock)
                        result['tf_available'].append([clock,p.x,p.y,float(yaw(q))])
                elif (tr.header.frame_id,tr.child_frame_id)==('odom','base_footprint'):result['tf_odom_base'].append(row)
                if name=='/tf_static':result['static_tf'].append(dict(parent=tr.header.frame_id,child=tr.child_frame_id,xyz=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w]))
        elif name=='/scan':result['scans'].append(m)
        elif name=='/episode/state':result['states'].append(dict(receipt_ns=receipt,**json.loads(m.data)))
        elif name=='/initialpose':result['initial_pose'].append(dict(frame=m.header.frame_id,pose=pose(m.pose.pose)))
        elif name=='/particle_cloud':
            weights=np.array([p.weight for p in m.particles]);x=np.array([p.pose.position.x for p in m.particles]);y=np.array([p.pose.position.y for p in m.particles])
            if weights.sum()>0:
                weights/=weights.sum();mx=np.sum(x*weights);my=np.sum(y*weights)
                result['particles'].append([stamp(m.header.stamp),float(np.sqrt(np.sum(weights*(x-mx)**2))),float(np.sqrt(np.sum(weights*(y-my)**2)))])
    for key in ('odom','gt','amcl','tf_map_odom','tf_odom_base','gt3d','covariance','particles','wheel','tf_available'):
        result[key]=np.array(sorted(result[key],key=lambda r:r[0]))
    return result


class MapField:
    def __init__(self):
        from PIL import Image
        cfg=yaml.safe_load((ROOT/'maps/hall_01.yaml').read_text())
        self.image=np.array(Image.open(ROOT/'maps'/cfg['image']))
        self.res=cfg['resolution'];self.origin=np.array(cfg['origin'][:2]);self.h,self.w=self.image.shape
        self.dist=np.minimum(distance_transform_edt(self.image>=65)*self.res,2.)

    def distances(self,points,linear=True):
        col=(points[:,0]-self.origin[0])/self.res-.5
        row=self.h-.5-(points[:,1]-self.origin[1])/self.res
        # Biên ô là origin..origin+N*resolution, không phải tâm pixel đầu/cuối.
        result=map_coordinates(self.dist,[row,col],order=1 if linear else 0,mode='nearest')
        outside=(points[:,0]<self.origin[0])|(points[:,0]>=self.origin[0]+self.w*self.res)|\
                (points[:,1]<self.origin[1])|(points[:,1]>=self.origin[1]+self.h*self.res)
        result[outside]=2.
        return result


def transform_points(T,points):
    c,s=np.cos(T[2]),np.sin(T[2]);return points@np.array([[c,s],[-s,c]])+T[:2]


def world_scan(scan,bag):
    t=stamp(scan.header.stamp);data=bag['gt3d'];i=np.argmin(abs(data[:,0]-t));g=data[i]
    if abs(g[0]-t)>.015:return None,None
    ranges=np.array(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    valid=np.isfinite(ranges)&(ranges>=.12)&(ranges<scan.range_max)
    points=np.column_stack([ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())])
    # TF static được kiểm tra: footprint->base z=.0625; base->laser (.1,0,.18).
    world=Rotation.from_quat(g[4:]).apply(points+np.array([.1,0,.2425]))+g[1:4]
    return world,valid


def registration(bag,field,out):
    # Chỉ scan trước chuyển động, GT gần spawn; không dùng một pose AMCL nào để fit.
    scans=[]
    for sec in (1.5,2.5,3.5):
        scan=min(bag['scans'],key=lambda s:abs(stamp(s.header.stamp)-sec))
        gt=interpolate_pose(bag['gt'],[stamp(scan.header.stamp)])[0]
        if not np.isfinite(gt).all() or np.hypot(*gt[:2])>.002:raise ValueError('Registration requires independently known stationary spawn')
        points,_=world_scan(scan,bag);scans.append(points[:,:2])
    fits=[]
    for points in scans:
        fit=least_squares(lambda p:field.distances(transform_points(p,points)),[.04,.01,0],
              bounds=([-.2,-.15,-.02],[.2,.15,.02]),loss='soft_l1',f_scale=.05,diff_step=.001,max_nfev=200)
        fits.append(fit.x)
    T=np.median(fits,axis=0);points=np.concatenate(scans)
    # Sai số raster không thể biến thành độ chính xác mm chỉ vì nhiều scan.
    result=dict(world_to_map=T.tolist(),map_to_world=inverse(T).tolist(),
       method='Static world GT spawn + raw laser + fixed extrinsics against existing occupancy map, no AMCL pose fit',
       independent_scan_fits=[v.tolist() for v in fits],scan_stamps_s=[stamp(min(bag['scans'],key=lambda s:abs(stamp(s.header.stamp)-sec)).header.stamp) for sec in (1.5,2.5,3.5)],
       map_resolution_m=field.res,conservative_translation_resolution_bound_m=.05,
       conservative_rotation_resolution_bound_rad=.0036,
       rms_before_m=float(np.sqrt(np.mean(field.distances(points)**2))),
       rms_after_m=float(np.sqrt(np.mean(field.distances(transform_points(T,points))**2))),
       status='Independent static registration with raster/map uncertainty; not exact survey ground truth',
       fixed_for_all_episodes=True)
    save_json(out/'REGISTRATION.json',result)
    fig,ax=plt.subplots(figsize=(12,4))
    ax.imshow(field.image,cmap='gray',origin='upper',extent=[field.origin[0],field.origin[0]+field.w*field.res,field.origin[1],field.origin[1]+field.h*field.res])
    ax.scatter(points[:,0],points[:,1],s=2,label='Static world endpoints, nominal identity',alpha=.5)
    p=transform_points(T,points);ax.scatter(p[:,0],p[:,1],s=2,label='Independent fixed registration',alpha=.6)
    ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Map raster versus stationary world laser; no AMCL fit');ax.legend();fig.tight_layout();fig.savefig(out/'map_registration.png',dpi=180);plt.close(fig)
    return result


def plots(traces,out,tag,all_page=False):
    # Ba lượt verification + outlier cũ; các lượt còn lại xuất CSV đầy đủ.
    chosen=traces if all_page else [v for v in traces if '/verification/' in v['path'] or 'fixed3_integration_20261030/PID/episode_00001' in v['path']]
    for mode in ('time','progress'):
        fig,axes=plt.subplots(len(chosen),3,figsize=(15,2.7*len(chosen)),squeeze=False)
        for axrow,item in zip(axes,chosen):
            x=item['t']-item['t'][0] if mode=='time' else item['g'][:,0]
            for k,ax in enumerate(axrow):
                ax.plot(x,item['error'][:,k]*(180/np.pi if k==2 else 100),lw=1.)
                ax.axhline(0,color='black',lw=.5);ax.grid(alpha=.2)
                ax.set_ylabel(['Along X error (cm)','Lateral Y error (cm)','Yaw error (deg)'][k])
                ax.set_xlabel('Simulation time since first shared pose (s)' if mode=='time' else 'GT world progress X (m)')
                if mode=='progress':
                    for center in obstacle_positions():ax.axvline(center,color='red',alpha=.15,lw=.7)
            axrow[0].set_title(item['label'],fontsize=9)
        fig.suptitle(f'Localization estimate minus physical GT at common timestamp — {tag}',fontsize=12)
        fig.tight_layout(rect=[0,0,1,.97]);fig.savefig(out/f'errors_{mode}.png',dpi=180);fig.savefig(out/f'errors_{mode}.svg');plt.close(fig)
    if not all_page:
        for i in range(0,len(traces),4):
            page=out/f'page_{i//4+1}';page.mkdir(exist_ok=True)
            plots(traces[i:i+4],page,tag,all_page=True)


def analyze_old(out,T,tag):
    paths=list((ROOT/'rl_training/runs/fixed3_integration_20261030').glob('*/episode_*/telemetry.jsonl'))
    paths+=list((ROOT/'rl_training/runs/pid_measured_tuning_20261030').glob('*/PID/episode_*/telemetry.jsonl'))
    paths+=list((ROOT/'rl_training/runs/mission_audit_20261030/integration_PID').glob('PID/episode_*/telemetry.jsonl'))
    out.mkdir(exist_ok=True);summaries=[];traces=[];increments=[]
    centers=np.array(obstacle_positions())
    for p in sorted(paths):
        rows=read_rows(p);a=np.array([[r['amcl_stamp_s'],r['amcl_x'],r['amcl_y'],r['amcl_yaw']] for r in rows])
        g=np.array([[r['gt_stamp_s'],r['ground_truth_x'],r['ground_truth_y'],r['ground_truth_yaw']] for r in rows])
        t,a,g,e=aligned_errors(a,g,T)
        label=str(p.parent.relative_to(ROOT/'rl_training/runs'));traces.append(dict(path=str(p),label=label,t=t,g=g,error=e))
        start=np.median(e[t<t[0]+.5],axis=0);end=np.median(e[t>t[-1]-.2],axis=0)
        d=e[1:]-e[:-1];d[:,2]=wrap(d[:,2]);ds=np.linalg.norm(np.diff(g[:,:2],axis=0),axis=1)
        near=np.min(np.minimum(abs(g[1:,0,None]-centers),abs(g[1:,0,None]+.24362-centers)),axis=1)<=.15
        for name,mask in [('near_bump_or_caster',near),('outside_windows',~near)]:
            increments.append(dict(episode=label,region=name,seconds=float(np.diff(t)[mask].sum()),distance_m=float(ds[mask].sum()),
                longitudinal_net_change_m=float(d[mask,0].sum()),longitudinal_abs_change_m=float(np.abs(d[mask,0]).sum()),
                lateral_abs_change_m=float(np.abs(d[mask,1]).sum()),yaw_abs_change_rad=float(np.abs(d[mask,2]).sum())))
        summaries.append(dict(episode=label,samples=len(t),start_x_m=start[0],start_y_m=start[1],start_yaw_rad=start[2],
            end_x_m=end[0],end_y_m=end[1],end_yaw_rad=end[2],drift_x_m=end[0]-start[0],drift_y_m=end[1]-start[1],
            rms_x_m=np.sqrt(np.mean(e[:,0]**2)),rms_y_m=np.sqrt(np.mean(e[:,1]**2)),rms_yaw_rad=np.sqrt(np.mean(e[:,2]**2)),
            max_time_skew_s=max(abs(r['amcl_stamp_s']-r['gt_stamp_s']) for r in rows),
            raw_sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
        name=label.replace('/','__');write_csv(out/(name+'.csv'),[dict(t_s=ti,s_world_x_m=gi[0],estimated_world_x_m=ai[0],estimated_world_y_m=ai[1],gt_x_m=gi[0],gt_y_m=gi[1],delta_x_m=ei[0],delta_y_m=ei[1],delta_yaw_rad=ei[2]) for ti,ai,gi,ei in zip(t,a,g,e)])
    write_csv(out/'episodes.csv',summaries);write_csv(out/'bump_vs_other.csv',increments);plots(traces,out,tag)
    return summaries


def analyze_bag(bag,field,reg,out):
    T=np.array(reg['map_to_world']);o=bag['odom'];times=o[:,0]
    g=interpolate_pose(bag['gt'],times);od=o[:,1:]
    mo=interpolate_pose(bag['tf_map_odom'],times,max_gap=2.)
    est=compose(T,compose(mo,od));initial=compose(bag['gt'][0,1:],inverse(o[0,1:]))
    ow=compose(initial,od);valid=np.isfinite(g).all(axis=1)&np.isfinite(est).all(axis=1)
    times,g,ow,est,od,mo=times[valid],g[valid],ow[valid],est[valid],od[valid],mo[valid]
    err=est-g;err[:,2]=wrap(err[:,2]);oe=ow-g;oe[:,2]=wrap(oe[:,2]);effect=est-ow;effect[:,2]=wrap(effect[:,2])
    write_csv(out/'tf_decomposition.csv',[dict(t_s=t,gt_x_m=gp[0],gt_y_m=gp[1],odom_world_x_m=op[0],odom_world_y_m=op[1],estimated_world_x_m=ep[0],estimated_world_y_m=ep[1],odom_error_x_m=de[0],odom_error_y_m=de[1],odom_error_yaw_rad=de[2],map_correction_x_m=ce[0],map_correction_y_m=ce[1],map_correction_yaw_rad=ce[2],final_error_x_m=fe[0],final_error_y_m=fe[1],final_error_yaw_rad=fe[2]) for t,gp,op,ep,de,ce,fe in zip(times,g,ow,est,oe,effect,err)])
    fig,axes=plt.subplots(3,2,figsize=(15,10))
    for k in range(3):
        scale=180/np.pi if k==2 else 100
        for j,x in enumerate((times-times[0],g[:,0])):
            ax=axes[k,j];ax.plot(x,oe[:,k]*scale,label='Wheel odometry - GT');ax.plot(x,effect[:,k]*scale,label='map->odom correction effect');ax.plot(x,err[:,k]*scale,label='Final TF map->base - GT',lw=2)
            if j:
                for c in obstacle_positions():ax.axvline(c,color='red',alpha=.15)
            ax.set(xlabel='GT progress X (m)' if j else 'Time (s)',ylabel=['Longitudinal (cm)','Lateral (cm)','Yaw (deg)'][k]);ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('One diagnostic episode: exact Cartesian decomposition, fixed independent registration');fig.tight_layout();fig.savefig(out/'odom_map_decomposition.png',dpi=180);fig.savefig(out/'odom_map_decomposition.svg');plt.close(fig)
    at=bag['amcl'][:,0];ag=interpolate_pose(bag['gt'],at);ae=compose(T,bag['amcl'][:,1:])-ag;ae[:,2]=wrap(ae[:,2])
    write_csv(out/'raw_amcl_errors.csv',[dict(t_s=t,x_m=e[0],y_m=e[1],yaw_rad=e[2]) for t,e in zip(at,ae) if np.isfinite(e).all()])
    scan_stamps=np.array([stamp(s.header.stamp) for s in bag['scans']]);tfst=bag['tf_map_odom'][:,0]
    offsets=[]
    for ts in tfst:
        nearest=scan_stamps[np.argmin(abs(scan_stamps-(ts-1.)))]
        offsets.append(ts-nearest)
    result=dict(topic_counts=bag['counts'],static_tf=bag['static_tf'],initial_pose=bag['initial_pose'],states=bag['states'],
      last_gt=g[-1].tolist(),last_odom_world=ow[-1].tolist(),last_estimated_world=est[-1].tolist(),
      last_odom_error=oe[-1].tolist(),last_map_correction_effect=effect[-1].tolist(),last_final_error=err[-1].tolist(),
      last_raw_amcl_error=ae[np.isfinite(ae).all(axis=1)][-1].tolist(),
      decomposition_max_numeric_error=float(np.max(np.abs(oe[:,:2]+effect[:,:2]-err[:,:2]))),
      map_odom_header_minus_scan_stamp_median_s=float(np.median(offsets)),
      odom_tf_max_gap_s=float(np.max(np.diff(bag['tf_odom_base'][:,0]))),
      gt_max_gap_s=float(np.max(np.diff(bag['gt'][:,0]))),
      amcl_publish_gaps_s=[float(np.min(np.diff(at))),float(np.median(np.diff(at))),float(np.max(np.diff(at)))])
    save_json(out/'BAG_FINDINGS.json',result)
    return result


def source_checks(bag,out):
    """Đối chiếu phương trình odom với encoder, tách sự kiện gờ/đoạn ngoài cửa sổ."""
    wheel=bag['wheel'];dt=np.diff(wheel[:,0],prepend=wheel[0,0]);dt=np.clip(dt,0,.1)
    v=.0625*(wheel[:,1]+wheel[:,2])/2
    w=.0625*(wheel[:,2]-wheel[:,1])/.34273666
    theta=np.cumsum(w*dt);mid=theta-w*dt/2
    integrated=np.column_stack([wheel[:,0],np.cumsum(v*dt*np.cos(mid)),np.cumsum(v*dt*np.sin(mid)),wrap(theta)])
    od=bag['odom'];reconstructed=interpolate_pose(integrated,od[:,0]);g=interpolate_pose(bag['gt'],od[:,0])
    valid=np.isfinite(reconstructed).all(axis=1)&np.isfinite(g).all(axis=1)
    od,g,reconstructed=od[valid],g[valid],reconstructed[valid]
    difference=reconstructed-od[:,1:];difference[:,2]=wrap(difference[:,2])
    e=od[:,1:]-g;e[:,2]=wrap(e[:,2]);delta=np.diff(e,axis=0);delta[:,2]=wrap(delta[:,2])
    ds=np.linalg.norm(np.diff(g[:,:2],axis=0),axis=1);duration=np.diff(od[:,0]);moving=ds/duration>.05
    centers=np.array(obstacle_positions());near=np.min(np.minimum(abs(g[1:,0,None]-centers),abs(g[1:,0,None]+.24362-centers)),axis=1)<=.15
    rows=[]
    for name,mask in [('near_bump_or_caster',near&moving),('outside_windows',~near&moving),('low_translation_or_in_place_rotation',~moving)]:
        distance=ds[mask].sum()
        rows.append(dict(region=name,seconds=float(duration[mask].sum()),distance_m=float(distance),
           net_yaw_drift_rad=float(delta[mask,2].sum()),abs_yaw_drift_rad=float(abs(delta[mask,2]).sum()),
           abs_yaw_drift_per_m=float(abs(delta[mask,2]).sum()/distance) if distance>1e-8 else None,
           net_odom_error_x_change_m=float(delta[mask,0].sum()),net_odom_error_y_change_m=float(delta[mask,1].sum())))
    write_csv(out/'odom_bump_vs_other.csv',rows)
    bag['delivery']={key:dict(median_s=float(np.median(v)),p95_s=float(np.percentile(v,95)),min_s=float(min(v)),max_s=float(max(v))) for key,v in bag['delivery'].items()}
    result=dict(encoder_reintegration_endpoint_minus_raw_odom=difference[-1].tolist(),
                encoder_reintegration_rms_error=np.sqrt(np.mean(difference**2,axis=0)).tolist(),
                configured_radius_m=.0625,collision_cylinder_radius_m=.0625,
                configured_separation_m=.34273666,collision_center_separation_m=2*(.1935-.0235),
                separation_difference_pct=100*(.34273666/.34-1),
                yaw_change_if_only_use_collision_track_rad=float(theta[-1]*(.34273666/.34-1)),
                sensor_delivery_relative_latest_recorded_clock=bag['delivery'],
                region_method='Moving >.05m/s; base or front caster x within .15m of SDF bump center; descriptive windows, not measured contact',
                regions=rows)
    save_json(out/'SOURCES_AND_TIMING.json',result)
    return result


def scan_observability(bag,field,reg,out):
    config=yaml.safe_load((ROOT/'src/nino_navigation/config/nav2.yaml').read_text())['amcl']['ros__parameters']
    fig,axes=plt.subplots(5,3,figsize=(14,13));rows=[]
    for j,target in enumerate((0.,3.,6.,9.,10.)):
        options=[]
        for scan in bag['scans']:
            t=stamp(scan.header.stamp);g=bag['gt3d'][np.argmin(abs(bag['gt3d'][:,0]-t))]
            angles=Rotation.from_quat(g[4:]).as_euler('xyz')
            if abs(g[1]-target)<.3 and max(abs(angles[:2]))<.02:
                options.append((abs(g[1]-target),scan,g,angles))
        if not options:continue
        _,scan,g,tilt=min(options,key=lambda v:v[0]);world,_=world_scan(scan,bag)
        base=compose(reg['world_to_map'],[g[1],g[2],tilt[2]])
        ranges=np.array(scan.ranges);bearings=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        step=max(1,(len(ranges)-1)//(config['max_beams']-1));indices=np.arange(0,len(ranges),step)
        indices=indices[np.isfinite(ranges[indices])&(ranges[indices]>=.12)&(ranges[indices]<scan.range_max)]
        r,ang=ranges[indices],bearings[indices]
        def score(candidate):
            laser=compose(candidate,[.1,0,0]);points=np.column_stack([laser[0]+r*np.cos(laser[2]+ang),laser[1]+r*np.sin(laser[2]+ang)])
            dist=field.distances(points,linear=False)
            p=config['z_hit']*np.exp(-dist**2/(2*config['sigma_hit']**2))+config['z_rand']/scan.range_max
            return 1+np.sum(p**3)
        widths=[]
        for k,offset in enumerate((np.linspace(-.8,.8,321),np.linspace(-.3,.3,241),np.linspace(-.15,.15,241))):
            scores=[]
            for d in offset:
                candidate=base.copy();candidate[k]+=d;scores.append(score(candidate))
            scores=np.array(scores);scores/=scores.max();good=offset[scores>=.95]
            width=float(np.ptp(good)) if len(good) else 0.;widths.append(width)
            axes[j,k].plot(offset*(180/np.pi if k==2 else 1),scores)
            axes[j,k].axhline(.95,color='gray',ls='--',lw=.8)
            axes[j,k].set(xlabel=['X shift (m)','Y shift (m)','Yaw shift (deg)'][k],ylabel=f'X={g[1]:.2f} m\nNormalized score',ylim=(.4,1.02));axes[j,k].grid(alpha=.2)
        endpoint_endwall=np.minimum(abs(world[:,0]+2),abs(world[:,0]-12))<.08
        endpoint_sidewall=abs(abs(world[:,1])-2)<.08
        rows.append(dict(scan_stamp_s=stamp(scan.header.stamp),world_x_m=float(g[1]),roll_rad=float(tilt[0]),pitch_rad=float(tilt[1]),
             selected_beams=len(indices),x_score95_width_m=widths[0],y_score95_width_m=widths[1],yaw_score95_width_rad=widths[2],
             end_wall_endpoints_pct=float(100*endpoint_endwall.mean()),side_wall_endpoints_pct=float(100*endpoint_sidewall.mean())))
    fig.suptitle('Recorded scan: likelihood-field score slices; 95% score width is NOT a confidence interval')
    fig.tight_layout(rect=[0,0,1,.97]);fig.savefig(out/'hall_scan_observability.png',dpi=180);fig.savefig(out/'hall_scan_observability.svg');plt.close(fig)
    write_csv(out/'scan_observability.csv',rows)
    # Toàn bộ scan: endpoints chạm mặt sàn do roll/pitch so với map 2D.
    scan_rows=[]
    for scan in bag['scans']:
        world,_=world_scan(scan,bag)
        if world is None or len(world)==0:continue
        t=stamp(scan.header.stamp);g=bag['gt3d'][np.argmin(abs(bag['gt3d'][:,0]-t))];angles=Rotation.from_quat(g[4:]).as_euler('xyz')
        scan_rows.append(dict(t_s=t,world_x_m=float(g[1]),roll_rad=float(angles[0]),pitch_rad=float(angles[1]),
           floor_endpoints_pct=float(100*np.mean(abs(world[:,2])<.04))))
    write_csv(out/'scan_floor_endpoints.csv',scan_rows)
    return rows


def tf_timing(bag,reg,out,telemetry_folder=None):
    o=bag['odom'];t=o[:,0];g=interpolate_pose(bag['gt'],t);T=reg['map_to_world']
    mo=interpolate_pose(bag['tf_map_odom'],t,max_gap=2.)
    current=compose(T,compose(mo,o[:,1:]))
    # Giữ correction đã được nhận ở thời điểm đó; không lấy correction tương lai.
    available=bag['tf_available'];i=np.searchsorted(available[:,0],t,side='right')-1
    causal=compose(T,compose(available[np.maximum(i,0),1:],o[:,1:]));causal[i<0]=np.nan
    valid=np.isfinite(current).all(axis=1)&np.isfinite(causal).all(axis=1)&np.isfinite(g).all(axis=1)
    errors=current[valid]-g[valid];errors[:,2]=wrap(errors[:,2])
    alternate=causal[valid]-g[valid];alternate[:,2]=wrap(alternate[:,2])
    ar=bag['amcl'];ag=interpolate_pose(bag['gt'],ar[:,0]);raw=compose(T,ar[:,1:])-ag;raw[:,2]=wrap(raw[:,2])
    fig,axes=plt.subplots(3,1,figsize=(14,9))
    for k,ax in enumerate(axes):
        scale=180/np.pi if k==2 else 100
        ax.plot(t[valid],errors[:,k]*scale,label='Existing TF interpolation (recorded header timing)')
        ax.plot(t[valid],alternate[:,k]*scale,label='Offline latest available correction; same odom/trajectory',alpha=.7)
        ax.plot(ar[:,0],raw[:,k]*scale,'.',label='Published /amcl_pose at scan stamp',ms=3)
        ax.set(xlabel='Absolute simulation time (s)',ylabel=['X error (cm)','Y error (cm)','Yaw error (deg)'][k]);ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.suptitle('TF timing diagnostic only: offline alternate is NOT a controller rerun')
    fig.tight_layout();fig.savefig(out/'tf_timing_vs_raw_amcl.png',dpi=180);plt.close(fig)
    # Chứng minh reconstruction đúng pose thực mà controller đã đọc qua TF.
    rawfile=next((telemetry_folder or out/'capture_episode/PID').glob('episode_*/telemetry.jsonl'))
    telemetry=read_rows(rawfile);tt=np.array([r['amcl_stamp_s'] for r in telemetry])
    po=interpolate_pose(bag['odom'],tt);pm=interpolate_pose(bag['tf_map_odom'],tt,max_gap=2.)
    expected=compose(pm,po);logged=np.array([[r['amcl_x'],r['amcl_y'],r['amcl_yaw']] for r in telemetry])
    difference=expected-logged;difference[:,2]=wrap(difference[:,2]);difference=difference[np.isfinite(difference).all(axis=1)]
    result=dict(recorded_tf_rms_error=np.sqrt(np.mean(errors**2,axis=0)).tolist(),
       causal_latest_correction_offline_rms=np.sqrt(np.mean(alternate**2,axis=0)).tolist(),
       reconstruction_vs_actual_telemetry_rms=np.sqrt(np.mean(difference**2,axis=0)).tolist(),
       reconstruction_vs_actual_telemetry_max=np.max(abs(difference),axis=0).tolist(),
       meaning='Postdated TF headers reproduce delayed correction effect; alternate only reprocesses captured trajectory, not a closed-loop validation',
       runtime_config_changed=False)
    save_json(out/'TF_TIMING.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--bag-dir',type=Path,required=True)
    parser.add_argument('--registration',required=True,help='Cùng file đăng ký độc lập dùng cho runner/audit/metrics')
    args=parser.parse_args()
    evidence=load_registration(args.registration,ROOT/'maps/hall_01.yaml',required=True)
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    field=MapField();bag=extract_bag(args.bag_dir)
    reg=dict(map_to_world=evidence['map_to_world'],world_to_map=inverse(evidence['map_to_world']).tolist(),registration=evidence)
    save_json(out/'USED_REGISTRATION.json',evidence)
    analyze_old(out/'old_registered',reg['map_to_world'],'shared independent registration; source/raster uncertainty retained')
    findings=analyze_bag(bag,field,reg,out)
    sources=source_checks(bag,out);scans=scan_observability(bag,field,reg,out)
    timing=tf_timing(bag,reg,out,args.bag_dir.parent/'capture_episode/PID')
    print(json.dumps(dict(registration=reg,bag_findings=findings,sources=sources,scans=scans,timing=timing),indent=2))


if __name__=='__main__':main()
