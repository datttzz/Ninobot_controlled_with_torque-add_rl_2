"""Đối sánh log bất kỳ theo nhãn; không điều khiển robot hoặc huấn luyện model."""
import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
from datetime import datetime, timezone
import platform
import statistics
import zipfile

import numpy as np
import yaml

# Các alias chỉ đổi tên/unit đã biết; không tái tạo cảm biến không được ghi.
SCALARS = {
    'cross_track': ('cross_track', 'cross_track_error_m', 'cross_track_error', 'e_y'),
    'heading_error': ('heading_error', 'heading_error_rad', 'e_heading'),
    'progress': ('progress', 'progress_m', 'path_progress_m'),
    'amcl_x': ('amcl_x', 'amcl_x_m', 'x_amcl'),
    'amcl_y': ('amcl_y', 'amcl_y_m', 'y_amcl'),
    'amcl_yaw': ('amcl_yaw', 'amcl_yaw_rad'),
    'gt_x': ('ground_truth_x', 'ground_truth_x_m', 'gt_x'),
    'gt_y': ('ground_truth_y', 'ground_truth_y_m', 'gt_y'),
    'roll': ('roll', 'roll_rad'), 'pitch': ('pitch', 'pitch_rad'),
}
VECTORS = {
    'acceleration': (3, ('acceleration',), [('ax', 'ay', 'az'), ('a_x', 'a_y', 'a_z'),
                       ('acceleration_x', 'acceleration_y', 'acceleration_z')]),
    'gyro': (3, ('gyro', 'angular_velocity'), [('gyro_x','gyro_y','gyro_z'),
                       ('angular_velocity_x','angular_velocity_y','angular_velocity_z')]),
    'wheel_velocity': (2, ('wheel_velocity','wheel_velocities'), [('wheel_left_radps','wheel_right_radps'),
                       ('left_wheel_velocity','right_wheel_velocity')]),
    'wheel_target': (2, ('wheel_target','target_velocity'), [('target_left_radps','target_right_radps')]),
    'base_torque': (2, ('base_torque',), [('pi_torque_left_nm','pi_torque_right_nm')]),
    'residual_applied': (2, ('residual_applied',), [('residual_left_nm','residual_right_nm')]),
    'total_torque': (2, ('total_torque',), [('total_torque_left_nm','total_torque_right_nm')]),
}
FLAGS = {'saturated': ('saturated','torque_saturated','saturation_flags'),
         'slew_limited': ('slew_limited','slew_rate_limited','slew_rate_flags')}
METRICS = {
    'duration_s': 's', 'completion_time_s': 's',
    'cross_track_rmse_sample_m': 'm', 'cross_track_rmse_time_m': 'm',
    'cross_track_peak_m': 'm', 'cross_track_iae_m_s': 'm*s',
    'cross_track_ise_m2_s': 'm^2*s', 'cross_track_itae_m_s2': 'm*s^2',
    'within_max_deviation_pct': '%', 'heading_rmse_rad': 'rad',
    'acceleration_rms_m_s2': 'm/s^2', 'acceleration_z_rms_m_s2': 'm/s^2',
    'acceleration_peak_m_s2': 'm/s^2', 'sampled_jerk_rms_m_s3': 'm/s^3',
    'roll_peak_rad': 'rad', 'pitch_peak_rad': 'rad', 'gyro_rms_rad_s': 'rad/s',
    'wheel_tracking_rmse_rad_s': 'rad/s', 'amcl_gt_position_rmse_m': 'm',
    'base_torque_rms_nm': 'Nm', 'residual_torque_rms_nm': 'Nm',
    'total_torque_rms_nm': 'Nm', 'total_torque_peak_nm': 'Nm',
    'final_left_torque_nm': 'Nm', 'final_right_torque_nm': 'Nm',
    'torque_squared_integral_nm2_s': 'Nm^2*s', 'sampled_torque_rate_rms_nm_s': 'Nm/s',
    'saturation_pct': '%', 'slew_limited_pct': '%', 'reward_total': 'reward',
}
OUTCOMES = {'SUCCEEDED','FAILED','TIME_LIMIT','INCOMPLETE'}


def finite(value):
    if value is None or value == '':
        return None
    if isinstance(value,bool):raise ValueError('Boolean is not a numeric sensor sample')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'Non-finite numeric value: {value}')
    return number


def pick(row, names):
    return next((row[n] for n in names if n in row and row[n] is not None and row[n] != ''), None)


def vector(row, size, names, components):
    value = pick(row,names)
    if value is not None:
        value = json.loads(value) if isinstance(value,str) else value
        if not isinstance(value,(list,tuple)) or len(value)!=size:
            raise ValueError(f'{names[0]} must have {size} components')
        values=[finite(v) for v in value]
        return values if all(v is not None for v in values) else None
    for fields in components:
        if all(pick(row,(k,)) is not None for k in fields):
            return [finite(row[k]) for k in fields]
    return None


def flag_value(value):
    if value is None or value=='': return None
    if isinstance(value,str):
        try: value=json.loads(value.lower())
        except json.JSONDecodeError: raise ValueError(f'Invalid flag: {value}')
    values=value if isinstance(value,list) else [value]
    if not values or any(v not in (True,False,0,1) for v in values):
        raise ValueError(f'Flags must be boolean/0/1: {value}')
    return float(np.mean(values))


def canonical(row):
    out={key:finite(pick(row,names)) for key,names in SCALARS.items()}
    for key,(size,names,components) in VECTORS.items():
        out[key]=vector(row,size,names,components)
    for key,names in FLAGS.items(): out[key]=flag_value(pick(row,names))
    time_name=next((k for k in ('simulation_time','time_s','timestamp','timestamp_s','timestamp_ns')
                    if pick(row,(k,)) is not None),None)
    if time_name is None: raise ValueError('Missing sample timestamp')
    out['t']=finite(row[time_name])*(1e-9 if time_name=='timestamp_ns' else 1)
    out['relative_episode_time']=time_name=='simulation_time'
    out['dt']=finite(pick(row,('step_duration','dt_s','dt')))
    outcome=pick(row,('outcome','status'))
    outcome=str(outcome).upper() if outcome is not None else None
    if outcome and outcome not in OUTCOMES: raise ValueError(f'Unknown outcome: {outcome}')
    out.update(outcome=outcome,reason=str(pick(row,('reason','stop_reason')) or ''),
               reference_hash=pick(row,('reference_geometry_sha256',)),
               planning_requests=finite(pick(row,('planning_requests',))))
    terms=row.get('reward_terms')
    if isinstance(terms,str) and terms: terms=json.loads(terms)
    if terms:
        if not isinstance(terms,dict): raise ValueError('reward_terms must be a dictionary')
        out['reward_terms']={str(k):finite(v) for k,v in terms.items()}
        if any(v is None for v in out['reward_terms'].values()): raise ValueError('Missing reward component')
    else:
        out['reward_terms']={k[7:]:finite(v) for k,v in row.items() if k.startswith('reward_') and k!='reward_total' and v not in (None,'')}
    out['reward']=finite(pick(row,('reward','step_reward')))
    seed=row.get('seed_label')
    out['seed_label']=seed if isinstance(seed,int) and not isinstance(seed,bool) else None
    out['ground_truth_frame']=row.get('ground_truth_frame')
    return out


def read_rows(path):
    if path.suffix=='.jsonl':
        raw=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    else:
        with path.open(newline='') as stream:
            raw=list(csv.DictReader(line for line in stream if not line.startswith('#')))
    if not raw: raise ValueError('Empty telemetry file')
    if not any(k in raw[0] for k in ('simulation_time','time_s','timestamp','timestamp_s','timestamp_ns')):
        raise ValueError('CSV is not step telemetry (monitor.csv contains only episode summaries)')
    partitions={}
    for row in raw:
        key=str(pick(row,('episode_id','episode_index','episode')) or '0')
        partitions.setdefault(key,[]).append(row)
    return [(key,[canonical(r) for r in rows]) for key,rows in partitions.items()]


def timeline(rows):
    if len(rows)<2: raise ValueError('At least two samples required')
    t=np.array([r['t'] for r in rows],dtype=float)
    interval=np.diff(t,prepend=t[0])
    if rows[0]['relative_episode_time'] and t[0]<0: raise ValueError('Negative episode-relative time')
    if np.any(interval<0): raise ValueError('Sample timestamps decrease')
    if any(r['outcome'] not in (None,'INCOMPLETE') for r in rows[:-1]):
        raise ValueError('Terminal outcome appears before final sample; split episodes explicitly')
    weights=np.array([r['dt'] if r['dt'] is not None else interval[i] for i,r in enumerate(rows)])
    if np.any(weights<0): raise ValueError('Negative sample duration')
    for i in range(1,len(t)):
        if interval[i]==0:
            if i!=len(t)-1 or rows[i]['outcome'] in (None,'INCOMPLETE'):
                raise ValueError('Duplicate timestamp outside terminal sample')
            weights[i]=0
    if weights.sum()<=0 or t[-1]<=t[0]: raise ValueError('No elapsed sampling time')
    return t,weights


def signal(rows,key):
    return np.array([r.get(key) if r.get(key) is not None else np.nan for r in rows],dtype=float)


def matrix(rows,key,width):
    return np.array([r.get(key) if r.get(key) is not None else [np.nan]*width for r in rows],dtype=float)


def weighted(values,weights):
    valid=np.isfinite(values) & (weights>0)
    if not np.any(valid): return None
    return float(np.average(values[valid],weights=weights[valid]))


def rms(values,weights):
    mean=weighted(values**2,weights)
    return math.sqrt(mean) if mean is not None else None


def rate_rms(values,t):
    if values.shape[0]<2:return None
    delta=np.diff(t)
    valid=(delta>0) & np.all(np.isfinite(values[:-1]),axis=1) & np.all(np.isfinite(values[1:]),axis=1)
    if not np.any(valid):return None
    derivative=np.diff(values,axis=0)[valid]/delta[valid,None]
    return math.sqrt(float(np.average(np.sum(derivative**2,axis=1),weights=delta[valid])))


def describe(rows,max_deviation):
    t,w=timeline(rows)
    metrics={k:None for k in METRICS}
    coverage={}
    elapsed=t-t[0]
    analysis_time=t if rows[0]['relative_episode_time'] else elapsed
    duration=float(t[-1]) if rows[0]['relative_episode_time'] else float(elapsed[-1])
    metrics['duration_s']=duration
    outcome=rows[-1]['outcome'] or 'INCOMPLETE'
    if outcome=='SUCCEEDED': metrics['completion_time_s']=duration
    e=signal(rows,'cross_track');valid=np.isfinite(e)
    coverage['cross_track']=float(w[valid].sum()/w.sum())
    if np.any(valid):
        metrics['cross_track_rmse_sample_m']=float(np.sqrt(np.mean(e[valid]**2)))
        metrics['cross_track_rmse_time_m']=rms(e,w)
        metrics['cross_track_peak_m']=float(np.max(np.abs(e[valid])))
        if np.any(valid & (w>0)):
            metrics['cross_track_iae_m_s']=float(np.sum(np.abs(e[valid])*w[valid]))
            metrics['cross_track_ise_m2_s']=float(np.sum(e[valid]**2*w[valid]))
            metrics['cross_track_itae_m_s2']=float(np.sum(analysis_time[valid]*np.abs(e[valid])*w[valid]))
        mean=weighted((np.abs(e)<=max_deviation).astype(float)[valid],w[valid])
        metrics['within_max_deviation_pct']=100*mean if mean is not None else None
    heading=signal(rows,'heading_error')
    heading=np.arctan2(np.sin(heading),np.cos(heading))
    metrics['heading_rmse_rad']=rms(heading,w)
    acc=matrix(rows,'acceleration',3);acc_valid=np.all(np.isfinite(acc),axis=1)
    coverage['acceleration']=float(w[acc_valid].sum()/w.sum())
    magnitude=np.linalg.norm(acc,axis=1)
    metrics['acceleration_rms_m_s2']=rms(magnitude,w)
    metrics['acceleration_z_rms_m_s2']=rms(acc[:,2],w)
    if np.any(acc_valid):metrics['acceleration_peak_m_s2']=float(np.max(magnitude[acc_valid]))
    metrics['sampled_jerk_rms_m_s3']=rate_rms(acc,t)
    for key in ('roll','pitch'):
        v=signal(rows,key)
        if np.any(np.isfinite(v)):metrics[key+'_peak_rad']=float(np.nanmax(np.abs(v)))
    metrics['gyro_rms_rad_s']=rms(np.linalg.norm(matrix(rows,'gyro',3),axis=1),w)
    wheel=matrix(rows,'wheel_velocity',2);target=matrix(rows,'wheel_target',2)
    metrics['wheel_tracking_rmse_rad_s']=rms(np.sqrt(np.mean((wheel-target)**2,axis=1)),w)
    amcl=np.column_stack((signal(rows,'amcl_x'),signal(rows,'amcl_y')))
    gt=np.column_stack((signal(rows,'gt_x'),signal(rows,'gt_y')))
    metrics['amcl_gt_position_rmse_m']=rms(np.linalg.norm(amcl-gt,axis=1),w)
    mismatched_frames=any(r.get('ground_truth_frame') not in (None,'','map') for r in rows)
    if mismatched_frames:metrics['amcl_gt_position_rmse_m']=None
    for key,name in [('base_torque','base'),('residual_applied','residual'),('total_torque','total')]:
        values=matrix(rows,key,2);good=np.all(np.isfinite(values),axis=1)
        coverage[key]=float(w[good].sum()/w.sum())
        metrics[name+'_torque_rms_nm']=rms(np.sqrt(np.mean(values**2,axis=1)),w)
        if key=='total_torque' and np.any(good):
            metrics['total_torque_peak_nm']=float(np.nanmax(np.abs(values)))
            power=np.sum(values**2,axis=1)
            if np.any(good & (w>0)):
                metrics['torque_squared_integral_nm2_s']=float(np.sum(power[good]*w[good]))
            metrics['sampled_torque_rate_rms_nm_s']=rate_rms(values,t)
            if np.all(np.isfinite(values[-1])):
                metrics['final_left_torque_nm']=float(values[-1,0]);metrics['final_right_torque_nm']=float(values[-1,1])
    for key,metric in [('saturated','saturation_pct'),('slew_limited','slew_limited_pct')]:
        value=weighted(signal(rows,key),w)
        metrics[metric]=100*value if value is not None else None
    expected_terms={key for r in rows for key in r['reward_terms']}
    if all(r['reward'] is not None or (bool(r['reward_terms']) and set(r['reward_terms'])==expected_terms) for r in rows):
        metrics['reward_total']=sum(r['reward'] if r['reward'] is not None else sum(r['reward_terms'].values()) for r in rows)
    terms=defaultdict(float)
    for r in rows:
        for key,value in r['reward_terms'].items():terms[key]+=value
    term_keys=set(terms)
    terms={key:terms[key] if all(key in r['reward_terms'] for r in rows) else None for key in term_keys}
    references=sorted({r['reference_hash'] for r in rows if r['reference_hash']})
    notes=[]
    if mismatched_frames:notes.append('Ground truth and AMCL use different frames; position RMSE is N/A without registration')
    if any(v is None for v in terms.values()):notes.append('Reward components have incomplete sample coverage')
    if len(references)>1:notes.append('Reference geometry changed during episode')
    if any(r['planning_requests'] is not None and r['planning_requests']!=1 for r in rows):
        notes.append('Planning request count differs from one')
    if any(0<c<1 for c in coverage.values()):notes.append('Partial signal coverage: consult coverage fractions')
    if any(value is not None and not math.isfinite(value) for value in metrics.values()):
        raise ValueError('Computed metric is non-finite; inspect input units/range')
    return dict(outcome=outcome,reason=rows[-1]['reason'],samples=len(rows),metrics=metrics,coverage=coverage,
                integration_time_s=float(w.sum()),reference_hashes=references,reward_components=terms,notes=notes)


def read_json(path,default):
    return json.loads(path.read_text()) if path.exists() else default


def context_for(folder):
    for candidate in (folder,*folder.parents):
        if (candidate/'metadata.json').exists() or (candidate/'config.yaml').exists():
            return candidate
    return folder


def discover(folder):
    """Ưu tiên log nguyên bản; không đọc lại CSV báo cáo để tránh đếm trùng."""
    if (folder/'steps.jsonl').exists(): return [(folder/'steps.jsonl',folder)]
    for name in ('steps.csv','telemetry.csv','trajectory.csv'):
        if (folder/name).exists():return [(folder/name,folder)]
    episodes=sorted(p for p in folder.glob('episode_*') if p.is_dir())
    if episodes:
        found=[]
        for episode in episodes:
            if (episode/'steps.jsonl').exists():found.append((episode/'steps.jsonl',episode))
            else:
                csvs=sorted(p for p in episode.glob('*.csv') if p.name!='monitor.csv')
                preferred=next((episode/name for name in ('steps.csv','telemetry.csv','trajectory.csv') if (episode/name).exists()),None)
                if preferred:found.append((preferred,episode))
                elif len(csvs)>1:raise ValueError(f'Multiple CSV files in one episode; merge telemetry columns first: {episode}')
                elif csvs:found.append((csvs[0],episode))
                else:found.append((None,episode))
        return found
    csvs=sorted(p for p in folder.glob('*.csv') if p.name not in
                {'monitor.csv','episodes.csv','comparison_summary.csv','summary.csv'})
    if csvs:return [(p,folder) for p in csvs]
    children=[p for p in sorted(folder.iterdir()) if p.is_dir() and
              ((p/'metadata.json').exists() or list(p.glob('episode_*')))]
    if children:return [item for child in children for item in discover(child)]
    return []


def model_evidence(metadata):
    path=metadata.get('model')
    if not path:return None
    result={'path':path,'timesteps':None}
    p=Path(path)
    if not p.is_file():
        result['note']='Referenced model is unavailable; no timestep inferred from its name'
        return result
    try:
        # Đọc JSON thuần trong archive SB3; không unpickle, không nạp torch/model.
        with zipfile.ZipFile(p) as archive:
            result['timesteps']=json.loads(archive.read('data')).get('num_timesteps')
        result['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    except (ValueError,KeyError,OSError,zipfile.BadZipFile) as exc:
        result['note']='Unable to read checkpoint JSON: '+str(exc)
    return result


def windows_for(rows,episode,obstacles,radius,axis,max_deviation):
    result=[]
    values=signal(rows,axis)
    for center in obstacles:
        finite_pos=values[np.isfinite(values)]
        selected=np.flatnonzero(np.isfinite(values) & (np.abs(values-center)<=radius))
        item=dict(group=episode['group'],episode=episode['episode'],source=episode['source'],outcome=episode['outcome'],
                  obstacle=center,axis=axis,radius_m=radius,samples=len(selected),
                  coordinate_available=bool(len(finite_pos)),center_crossed=False,window_complete=False,
                  metrics=None)
        if len(finite_pos):
            item['center_crossed']=bool(np.min(finite_pos)<=center<=np.max(finite_pos))
            item['window_complete']=bool(np.min(finite_pos)<=center-radius and np.max(finite_pos)>=center+radius)
        if len(selected)>=2:
            # Cắt đúng phần khoảng thời gian có cả hai đầu mẫu nằm trong cửa sổ.
            # Không gán cả dt trước lúc xe vào cửa sổ cho vùng gờ.
            chosen=[]
            for j,i in enumerate(selected):
                r=rows[i].copy();r['outcome']=None
                r['dt']=rows[i]['t']-rows[i-1]['t'] if j and i==selected[j-1]+1 else 0.0
                chosen.append(r)
            if sum(r['dt'] for r in chosen)>0:
                m=describe(chosen,max_deviation)['metrics']
                item['metrics']={k:m[k] for k in ('cross_track_rmse_time_m','cross_track_peak_m',
                    'acceleration_rms_m_s2','acceleration_z_rms_m_s2','acceleration_peak_m_s2',
                    'total_torque_rms_nm','total_torque_peak_nm')}
        result.append(item)
    return result


def trace_data(rows,episode):
    # Giữ mọi mẫu của các tín hiệu vẽ; không đưa log text vào JavaScript thực thi.
    traces={'time':[r['t'] for r in rows]}
    for key in ['cross_track','heading_error','progress','amcl_x','gt_x','roll','pitch']:
        traces[key]=[r[key] for r in rows]
    for key,width in [('acceleration',3),('total_torque',2),('residual_applied',2),('gyro',3)]:
        traces[key+'_norm']=[float(np.linalg.norm(r[key])) if r[key] is not None else None for r in rows]
    return dict(group=episode['group'],episode=episode['episode'],outcome=episode['outcome'],data=traces)


def load_groups(specs,obstacles,radius,axis,max_deviation):
    groups={};audit=[];all_windows=[];traces=[];seen=set();contexts_seen=set()
    for label,folder in specs:
        group=groups.setdefault(label,dict(label=label,paths=[],contexts=[],episodes=[],startup_only_sessions=0,
                                           invalid_episodes=0,infrastructure_event_count=0))
        group['paths'].append(str(folder))
        entries=discover(folder)
        if not entries:
            audit.append(dict(group=label,kind='no_step_telemetry',source=str(folder),
                              detail='No per-step JSONL/CSV found; monitor.csv is only an episode summary'))
        ordinal=defaultdict(int)
        for path,episode_dir in entries:
            context=context_for(episode_dir)
            context_id=(label,str(context))
            if context_id not in contexts_seen:
                contexts_seen.add(context_id)
                metadata=read_json(context/'metadata.json',{})
                config=yaml.safe_load((context/'config.yaml').read_text()) if (context/'config.yaml').exists() else {}
                group['contexts'].append(dict(path=str(context),metadata=metadata,config=config or {},model=model_evidence(metadata)))
                recovery=context/'startup_recovery.jsonl'
                if recovery.exists():
                    for line in recovery.read_text().splitlines():
                        if line.strip():
                            group['infrastructure_event_count']+=1
                            audit.append(dict(group=label,kind='startup_retry',source=str(context),detail=json.loads(line)))
                events=context/'attempt_events.jsonl'
                if events.exists():
                    for line in events.read_text().splitlines():
                        if line.strip():
                            event=json.loads(line)
                            group['infrastructure_event_count']+=1
                            audit.append(dict(group=label,kind=event.get('event','runner_attempt'),
                                              source=str(events),detail=event))
                for fault in ([] if events.exists() else ['infrastructure_fault.json','exception.json','interrupted_model.zip']):
                    if (context/fault).exists():
                        group['infrastructure_event_count']+=1
                        audit.append(dict(group=label,kind='interruption' if fault.endswith('.zip') else fault,
                                          source=str(context/fault),detail=read_json(context/fault,{}) if fault.endswith('.json') else 'Interrupted snapshot present'))
            if path is None or not path.stat().st_size:
                group['startup_only_sessions']+=1
                audit.append(dict(group=label,kind='session_without_samples',source=str(episode_dir),
                                  detail='No trajectory recorded; not classified as a robot failure'))
                continue
            if path.resolve() in seen:
                raise ValueError(f'Telemetry supplied more than once (possibly overlapping input folders): {path}')
            seen.add(path.resolve())
            try:
                parsed=read_rows(path)
            except (ValueError,TypeError,json.JSONDecodeError) as exc:
                group['invalid_episodes']+=1
                ordinal[str(context)]+=1
                audit.append(dict(group=label,kind='invalid_telemetry',source=str(path),detail=str(exc)))
                continue
            metadata=read_json(context/'metadata.json',{})
            source_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
            for part,rows in parsed:
                episode_name=str(episode_dir.relative_to(folder)) if episode_dir!=folder else path.stem
                if len(parsed)>1:episode_name+=':'+part
                if episode_name=='.':episode_name=episode_dir.name
                seed=(metadata['seed']+ordinal[str(context)]) if metadata.get('mode') in ('baseline','evaluate') and isinstance(metadata.get('seed'),int) else None
                if isinstance(rows[-1].get('seed_label'),int):seed=rows[-1]['seed_label']
                ordinal[str(context)]+=1
                try:
                    data=describe(rows,max_deviation)
                except (ValueError,TypeError,KeyError) as exc:
                    group['invalid_episodes']+=1
                    audit.append(dict(group=label,kind='invalid_episode',source=str(path),episode=episode_name,detail=str(exc)))
                    continue
                episode=dict(group=label,episode=episode_name,source=str(path),source_sha256=source_sha256,seed_label=seed,**data)
                group['episodes'].append(episode)
                all_windows.extend(windows_for(rows,episode,obstacles,radius,axis,max_deviation))
                traces.append(trace_data(rows,episode))
                for note in data['notes']:
                    audit.append(dict(group=label,kind='episode_note',source=str(path),detail=note))
    return list(groups.values()),audit,all_windows,traces


def moments(values):
    values=[float(v) for v in values if v is not None and math.isfinite(v)]
    return dict(n=len(values),mean=statistics.mean(values) if values else None,
                std=statistics.stdev(values) if len(values)>1 else None)


def wilson(successes,n):
    if not n:return (None,None)
    z=1.959963984540054;p=successes/n;den=1+z*z/n
    center=(p+z*z/(2*n))/den
    margin=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/den
    return (100*max(0,center-margin),100*min(1,center+margin))


def aggregate(group):
    complete=[e for e in group['episodes'] if e['outcome'] in ('SUCCEEDED','FAILED','TIME_LIMIT')]
    n=len(complete);success=sum(e['outcome']=='SUCCEEDED' for e in complete)
    lo,hi=wilson(success,n)
    out=dict(label=group['label'],n_input_dirs=len(group['paths']),n_run_contexts=len(group['contexts']),n_completed=n,n_success=success,
             n_failed=sum(e['outcome']=='FAILED' for e in complete),n_time_limit=sum(e['outcome']=='TIME_LIMIT' for e in complete),
             n_incomplete=sum(e['outcome']=='INCOMPLETE' for e in group['episodes']),
             n_invalid=group['invalid_episodes'],startup_only_sessions=group['startup_only_sessions'],
             infrastructure_events=group['infrastructure_event_count'],success_rate_pct=100*success/n if n else None,
             success_wilson95_low_pct=lo,success_wilson95_high_pct=hi,
             metrics={key:moments(e['metrics'].get(key) for e in complete) for key in METRICS})
    names=sorted({key for e in complete for key in e['reward_components']})
    out['reward_components']={key:moments(e['reward_components'].get(key) for e in complete) for key in names}
    return out


def comparability(groups,max_deviation):
    notices=[];critical=[]
    signatures=[];maps=[];rewards=[]
    keys=['goal_x','goal_y','max_path_deviation','residual_limit','step_seconds','max_episode_seconds']
    for g in groups:
        for ctx in g['contexts']:
            env=ctx['config'].get('environment',{})
            meta=ctx['metadata']
            signatures.append(json.dumps({key:env.get(key) for key in keys},sort_keys=True))
            if meta.get('mode')=='baseline' and any((e['metrics']['residual_torque_rms_nm'] or 0)>1e-9 for e in g['episodes']):
                critical.append(f"{g['label']}: baseline metadata conflicts with nonzero applied residual")
            maps.append(meta.get('map'))
            rewards.append(json.dumps(ctx['config'].get('reward'),sort_keys=True))
            if meta.get('mode') in ('train','smoke'):
                critical.append(f"{g['label']}: contains {meta['mode']} data; policy may change during collection")
            if meta.get('experiment_purpose')=='smoke':
                notices.append(f"{g['label']}: technical smoke test only; not evidence of controller quality or tuned PID")
            if meta.get('mode')=='evaluate' and not meta.get('deterministic',False):
                critical.append(f"{g['label']}: deterministic evaluation is not recorded")
            if env.get('max_path_deviation') is not None and not math.isclose(env['max_path_deviation'],max_deviation):
                notices.append(f"{g['label']}: analysis threshold {max_deviation} m differs from logged failure threshold {env['max_path_deviation']} m; logged outcomes are unchanged")
        complete=[e for e in g['episodes'] if e['outcome']!='INCOMPLETE']
        if not complete:notices.append(f"{g['label']}: no complete episodes available")
        if any(e['notes'] for e in g['episodes']):notices.append(f"{g['label']}: episode notes require review in audit")
    if len(set(signatures))>1:critical.append('Environment configs differ across inputs; compare these conditions explicitly')
    if len(set(maps))>1:critical.append('Recorded map paths differ across inputs')
    if any(v is None for v in maps):notices.append('Some inputs lack map metadata; matching conditions cannot be verified')
    if len(set(rewards))>1:notices.append('Reward definitions differ: reward totals/components are not a common performance scale')
    if len({len([e for e in g['episodes'] if e['outcome']!='INCOMPLETE']) for g in groups})>1:
        notices.append('Groups contain different numbers of completed episodes; inspect n and success uncertainty')
    notices.extend(['Same seed labels do not control Gazebo/AMCL noise; no paired-significance claim is made.',
                    'Matching map paths does not verify historical map/Nav2/actuator contents.',
                    'Spatial windows and sampled IMU peaks are not contact or obstacle-clearance sensors.',
                    'Obstacle centers must use the selected pose coordinate frame; no SDF-world to AMCL-map transform is inferred.',
                    'Failure/time-limit metrics remain included; incomplete/invalid/startup-only sessions are listed separately.',
                    'Torque is a command proxy; squared torque integral is not energy consumption.',
                    'Missing signals remain N/A; historical logs cannot recover unsaved sensors.'])
    return list(dict.fromkeys(notices)),list(dict.fromkeys(critical))


def write_csv(path,rows):
    if not rows:
        path.write_text('');return
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)


def tex_escape(value):
    mapping={'\\':r'\textbackslash{}','&':r'\&','%':r'\%','$':r'\$','#':r'\#','_':r'\_',
             '{':r'\{','}':r'\}','~':r'\textasciitilde{}','^':r'\textasciicircum{}'}
    return ''.join(mapping.get(c,c) for c in str(value))


def write_latex(path,summaries):
    selected=[('cross_track_rmse_time_m',100),('cross_track_peak_m',100),('completion_time_s',1),
              ('heading_rmse_rad',1),('acceleration_rms_m_s2',1),('total_torque_rms_nm',1)]
    lines=[r'% Requires \usepackage{booktabs}. Values are episode mean ± sample SD; -- = unavailable.',
           r'\begin{table*}[ht]',r'\centering',r'\caption{Recorded multi-model benchmark; all completed outcomes retained.}',
           r'\begin{tabular}{lrrrrrrr}',r'\toprule',
           r'Method & Success & $e_y$ RMS (cm) & Peak $|e_y|$ (cm) & $T_{success}$ (s) & Heading RMS (rad) & Acc. RMS ($m/s^2$) & Torque RMS (Nm) \\',r'\midrule']
    for s in summaries:
        fields=[tex_escape(s['label']),f"{s['n_success']}/{s['n_completed']}" if s['n_completed'] else '--']
        for key,scale in selected:
            m=s['metrics'][key]
            fields.append('--' if m['mean'] is None else f"${m['mean']*scale:.3f}"+
                          (f" \\pm {m['std']*scale:.3f}" if m['std'] is not None else '')+'$')
        lines.append(' & '.join(fields)+r' \\')
    lines += [r'\bottomrule',r'\end{tabular}',r'\label{tab:multi_model_benchmark}',r'\end{table*}',
              '% Success-only completion time; other means include failures/time limits.',
              '% Metric-specific n/coverage, signal provenance and confounders: comparison_summary.csv / benchmark_report.html.']
    path.write_text('\n'.join(lines)+'\n')


def run(specs,output,obstacles,radius,max_deviation,obstacle_axis,strict=False):
    groups,audit,windows,traces=load_groups(specs,obstacles,radius,obstacle_axis,max_deviation)
    notices,critical=comparability(groups,max_deviation)
    if strict and critical:raise ValueError('Strict comparability: '+'; '.join(critical))
    if obstacles and all(not w['coordinate_available'] for w in windows):
        notices.insert(0,f'No {obstacle_axis} coordinates recorded: obstacle windows are N/A. Progress is never silently treated as world X.')
    ordered=sorted(obstacles)
    if any(b-a<2*radius for a,b in zip(ordered,ordered[1:])):
        notices.append('Obstacle windows overlap; their samples/exposures are not independent and must not be summed as disjoint events')
    summaries=[aggregate(g) for g in groups]
    report=dict(analysis=dict(created_utc=datetime.now(timezone.utc).isoformat(),python=platform.python_version(),numpy=np.__version__),settings=dict(obstacle_x=obstacles,window_radius_m=radius,max_deviation_m=max_deviation,obstacle_axis=obstacle_axis),
                metric_units=METRICS,groups=groups,summaries=summaries,windows=windows,audit=audit,
                notices=notices,confounders=critical)
    output.mkdir(parents=True,exist_ok=False)
    flat=[]
    for s in summaries:
        row={k:v for k,v in s.items() if k not in ('metrics','reward_components')}
        for key,values in s['metrics'].items():row.update({key+'_'+stat:value for stat,value in values.items()})
        for key,values in s['reward_components'].items():row.update({'reward_component_'+key+'_'+stat:value for stat,value in values.items()})
        flat.append(row)
    write_csv(output/'comparison_summary.csv',flat)
    episodes=[]
    for group in groups:
        for e in group['episodes']:
            episodes.append(dict(group=e['group'],episode=e['episode'],source=e['source'],seed_label=e['seed_label'],
                                 outcome=e['outcome'],reason=e['reason'],samples=e['samples'],
                                 **e['metrics'],**{'coverage_'+k:v for k,v in e['coverage'].items()}))
    write_csv(output/'episode_metrics.csv',episodes)
    write_csv(output/'obstacle_windows.csv',[{**{k:v for k,v in w.items() if k!='metrics'},**(w['metrics'] or {})} for w in windows])
    (output/'benchmark_data.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    (output/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2,allow_nan=False))
    write_latex(output/'comparison_table.tex',summaries)
    from .benchmark_report import write_html
    write_html(output/'benchmark_report.html',report,traces)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',nargs='+',required=True,metavar='LABEL:PATH')
    parser.add_argument('--obstacle-x',type=float,nargs='+',default=[])
    parser.add_argument('--window-radius',type=float,default=.5)
    parser.add_argument('--max-deviation',type=float,default=.20)
    parser.add_argument('--obstacle-axis',choices=['x','ground-truth-x','progress'],default='x')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--strict-comparability',action='store_true')
    args=parser.parse_args(argv)
    if not math.isfinite(args.window_radius) or args.window_radius<=0 or not math.isfinite(args.max_deviation) or args.max_deviation<=0:
        parser.error('Window radius and max deviation must be finite and positive')
    if any(not math.isfinite(x) for x in args.obstacle_x):parser.error('Obstacle coordinates must be finite')
    specs=[]
    for item in args.runs:
        label,sep,path=item.partition(':')
        if not sep or not label.strip() or not path:parser.error('Use LABEL:/path/to/run for each input')
        folder=Path(path).expanduser().resolve()
        if not folder.is_dir():parser.error(f'Run directory does not exist: {folder}')
        specs.append((label.strip(),folder))
    axis={'x':'amcl_x','ground-truth-x':'gt_x','progress':'progress'}[args.obstacle_axis]
    try:
        report=run(specs,args.output_dir.expanduser().resolve(),args.obstacle_x,args.window_radius,args.max_deviation,axis,args.strict_comparability)
    except (ValueError,OSError,KeyError,TypeError) as exc:parser.error(str(exc))
    for s in report['summaries']:
        m=s['metrics']['cross_track_rmse_time_m']['mean']
        print(f"{s['label']}: success {s['n_success']}/{s['n_completed']}; time-weighted lateral RMS = {m if m is not None else 'N/A'} m")
    print('Report:',args.output_dir.expanduser().resolve()/'benchmark_report.html')
