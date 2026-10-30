"""Kiểm tra công thức, dữ liệu thiếu, lỗi hạ tầng và export nhiều model offline."""
from copy import deepcopy
import csv
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from rl_training.benchmark_suite import (canonical,describe,load_groups,main,model_evidence,run,windows_for)


def native(outcome='SUCCEEDED'):
    return [dict(simulation_time=t,step_duration=dt,cross_track=e,heading_error=0,
                 progress=p,acceleration=[3,0,0],total_torque=[2,-2],base_torque=[1,-1],
                 residual_applied=[1,-1],roll=.1,pitch=-.2,reward_terms={'progress':1,'terminal':0},
                 outcome=outcome if i==2 else None,reason='done',reference_geometry_sha256='same',planning_requests=1)
            for i,(t,dt,e,p) in enumerate([(1,1,1,0),(2,1,2,1),(4,2,0,2)])]


def save_run(folder,episodes,mode='evaluate',reward=None):
    folder.mkdir()
    (folder/'metadata.json').write_text(json.dumps(dict(mode=mode,seed=1,map='/map',deterministic=True)))
    import yaml
    (folder/'config.yaml').write_text(yaml.safe_dump({'environment':{'goal_x':11,'max_path_deviation':.2},'reward':reward or {'success':30}}))
    for i,rows in enumerate(episodes):
        e=folder/f'episode_{i:03d}';e.mkdir()
        (e/'steps.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    return folder


def test_time_weights_and_paper_formulas():
    m=describe([canonical(r) for r in native()],.2)
    values=m['metrics']
    assert values['cross_track_rmse_time_m']==pytest.approx(np.sqrt(5/4))
    assert values['cross_track_rmse_sample_m']==pytest.approx(np.sqrt(5/3))
    assert values['cross_track_iae_m_s']==3
    assert values['cross_track_ise_m2_s']==5
    assert values['cross_track_itae_m_s2']==5
    assert values['within_max_deviation_pct']==50
    assert values['acceleration_rms_m_s2']==3
    assert values['total_torque_rms_nm']==2
    assert values['torque_squared_integral_nm2_s']==32
    assert values['reward_total']==3
    assert m['reward_components']['progress']==3
    assert values['wheel_tracking_rmse_rad_s'] is None
    assert values['gyro_rms_rad_s'] is None
    assert values['saturation_pct'] is None
    assert values['amcl_gt_position_rmse_m'] is None


def test_duplicate_terminal_has_zero_time_weight():
    rows=native();rows[-1]['outcome']=None
    terminal=deepcopy(rows[-1]);terminal.update(outcome='SUCCEEDED',step_duration=.1)
    rows.append(terminal)
    result=describe([canonical(r) for r in rows],.2)
    assert result['integration_time_s']==4
    assert result['metrics']['cross_track_rmse_time_m']==pytest.approx(np.sqrt(5/4))


@pytest.mark.parametrize('mutation', ['negative_time','duplicate','early_outcome','nan'])
def test_invalid_timing_is_not_silently_scored(mutation):
    rows=native()
    if mutation=='negative_time':rows[-1]['simulation_time']=.5
    elif mutation=='duplicate':rows[1]['simulation_time']=1
    elif mutation=='early_outcome':rows[0]['outcome']='FAILED'
    else:rows[0]['simulation_time']=float('nan')
    with pytest.raises(ValueError):describe([canonical(r) for r in rows],.2)


def test_missing_first_point_is_not_zero_integrated_error():
    rows=[canonical(dict(time_s=i,cross_track=1 if i==0 else None,outcome='SUCCEEDED' if i==2 else None)) for i in range(3)]
    m=describe(rows,.2)['metrics']
    assert m['cross_track_rmse_time_m'] is None
    assert m['cross_track_iae_m_s'] is None
    assert m['cross_track_peak_m']==1


def test_csv_optional_sensors_and_flags(tmp_path):
    folder=tmp_path/'csv';folder.mkdir()
    fields=['episode_id','timestamp_ns','cross_track_error_m','ax','ay','az','gyro_x','gyro_y','gyro_z',
            'wheel_left_radps','wheel_right_radps','target_left_radps','target_right_radps',
            'amcl_x_m','amcl_y_m','ground_truth_x_m','ground_truth_y_m','saturation_flags','slew_rate_flags','outcome']
    with (folder/'steps.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for episode in [1,2]:
            for i in range(3):
                w.writerow(dict(zip(fields,[episode,(100+i)*1_000_000_000,.1,0,0,3,1,0,0,2,3,1,4,1,0,0,0,'[true,false]','true','SUCCEEDED' if i==2 else ''])))
    groups,audit,_,_=load_groups([('PID',folder)],[],.5,'amcl_x',.2)
    assert len(groups[0]['episodes'])==2
    m=groups[0]['episodes'][0]['metrics']
    assert m['wheel_tracking_rmse_rad_s']==1
    assert m['amcl_gt_position_rmse_m']==1
    assert m['gyro_rms_rad_s']==1
    assert m['saturation_pct']==50
    assert m['slew_limited_pct']==100
    assert m['duration_s']==2


def test_no_progress_to_world_x_substitution():
    rows=[canonical(r) for r in native()]
    episode={'group':'PI','episode':'e','source':'p','outcome':'SUCCEEDED'}
    physical=windows_for(rows,episode,[1],1,'amcl_x',.2)[0]
    progress=windows_for(rows,episode,[1],1,'progress',.2)[0]
    assert not physical['coordinate_available'] and physical['metrics'] is None
    assert progress['window_complete'] and progress['center_crossed'] and progress['metrics']


def test_disconnected_window_does_not_charge_outside_time():
    rows=native()+[deepcopy(native()[-1]) for _ in range(2)]
    for i,r in enumerate(rows):r.update(simulation_time=i,step_duration=1,progress=[0,1,3,1,0][i],outcome=None)
    e={'group':'PI','episode':'e','source':'p','outcome':'SUCCEEDED'}
    window=windows_for([canonical(r) for r in rows],e,[1],.1,'progress',.2)[0]
    assert window['samples']==2 and window['metrics'] is None


def test_multi_group_exports_preserve_failures_and_missing_values(tmp_path):
    specs=[]
    for i,label in enumerate(['PI','PID','PI+SAC (50k)','PI+SAC norm_&reward <script>']):
        folder=save_run(tmp_path/f'run{i}',[native(),native('FAILED'),native(None)])
        (folder/'episode_startup').mkdir()
        (folder/'startup_recovery.jsonl').write_text(json.dumps({'reason':'No IMU before goal'})+'\n')
        specs.append((label,folder))
    output=tmp_path/'out'
    report=run(specs,output,[1],.5,.2,'amcl_x')
    assert len(report['summaries'])==4
    for s in report['summaries']:
        assert (s['n_completed'],s['n_success'],s['n_failed'],s['n_incomplete'])==(2,1,1,1)
        assert s['success_rate_pct']==50
        assert s['metrics']['gyro_rms_rad_s']['n']==0
        assert s['metrics']['completion_time_s']['n']==1
    assert len(list(csv.DictReader((output/'comparison_summary.csv').open())))==4
    assert len(list(csv.DictReader((output/'episode_metrics.csv').open())))==12
    html=(output/'benchmark_report.html').read_text()
    assert 'cdn.' not in html and 'fetch(' not in html and 'src="http' not in html
    assert '\\u003cscript>' in html
    assert 'norm\\_\\&reward' in (output/'comparison_table.tex').read_text()
    assert (output/'benchmark_data.json').is_file()
    with pytest.raises(FileExistsError):run(specs,output,[],.5,.2,'progress')


def test_invalid_episode_and_monitor_only_audited(tmp_path):
    run_dir=save_run(tmp_path/'bad',[native()])
    (run_dir/'episode_000/steps.jsonl').write_text('{broken')
    groups,audit,_,_=load_groups([('bad',run_dir)],[],.5,'progress',.2)
    assert groups[0]['invalid_episodes']==1 and audit[0]['kind']=='invalid_telemetry'
    monitor=tmp_path/'monitor';monitor.mkdir();(monitor/'monitor.csv').write_text('r,l,t\n10,20,30\n')
    groups,audit,_,_=load_groups([('summary',monitor)],[],.5,'progress',.2)
    assert not groups[0]['episodes'] and audit[0]['kind']=='no_step_telemetry'


def test_strict_comparability_refuses_training_and_reward_ablation_is_not_rejected(tmp_path):
    a=save_run(tmp_path/'a',[native()],mode='train')
    with pytest.raises(ValueError,match='policy may change'):run([('training',a)],tmp_path/'out',[],.5,.2,'progress',True)
    assert not (tmp_path/'out').exists()
    b=save_run(tmp_path/'b',[native()],reward={'success':1})
    c=save_run(tmp_path/'c',[native()],reward={'success':30})
    result=run([('norm',b),('original',c)],tmp_path/'good',[],.5,.2,'progress',True)
    assert any('Reward definitions differ' in n for n in result['notices'])


def test_duplicate_inputs_and_bad_cli_refused(tmp_path):
    folder=save_run(tmp_path/'run',[native()])
    with pytest.raises(ValueError,match='more than once'):run([('A',folder),('B',folder)],tmp_path/'out',[],.5,.2,'progress')
    with pytest.raises(SystemExit):main(['--runs','no colon','--output-dir',str(tmp_path/'x')])


def test_model_steps_read_from_json_not_filename(tmp_path):
    model=tmp_path/'misleading_50000.zip'
    with zipfile.ZipFile(model,'w') as z:z.writestr('data',json.dumps({'num_timesteps':70000}))
    evidence=model_evidence({'model':str(model)})
    assert evidence['timesteps']==70000


def test_partial_reward_component_not_fabricated_as_full_return():
    rows=native()
    rows[0]['reward_terms'].pop('terminal')
    m=describe([canonical(r) for r in rows],.2)
    assert m['metrics']['reward_total'] is None
    assert m['reward_components']['terminal'] is None
    assert any('Reward components' in note for note in m['notes'])


def test_repeated_label_merges_distinct_runs(tmp_path):
    a=save_run(tmp_path/'a',[native()]);b=save_run(tmp_path/'b',[native('TIME_LIMIT')])
    result=run([('PI',a),('PI',b)],tmp_path/'result',[],.5,.2,'progress')
    s=result['summaries'][0]
    assert len(result['summaries'])==1
    assert (s['n_input_dirs'],s['n_run_contexts'],s['n_completed'],s['n_time_limit'])==(2,2,2,1)
