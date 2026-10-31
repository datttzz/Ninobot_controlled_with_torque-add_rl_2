"""Nguồn registration tổng hợp chỉ dùng cho test, không phải hiệu chuẩn robot."""
import json
import math
from pathlib import Path
import pytest
from rl_training.frame_registration import (ROOT,CONVENTION,asset_hashes,digest,load_registration,registration_from_metadata)


def write_registration(tmp_path,T=(.2,-.1,.3)):
    source=tmp_path/'synthetic_source.txt';source.write_text('Known independent synthetic frame fixture; not experimental data')
    doc=dict(version=1,convention=CONVENTION,frames=dict(source='map',target='world',robot_point='base_footprint'),
       map_to_world=list(T),assets=asset_hashes(ROOT/'maps/hall_01.yaml'),
       source=dict(independent_of_amcl=True,method='synthetic known transform fixture',files=[dict(path=str(source),sha256=digest(source))]),
       verification=dict(status='verified_independent_estimate',checks=['Known fixture only']))
    p=tmp_path/'registration.json';p.write_text(json.dumps(doc));return p


def test_nonidentity_source_validation_and_explicit_identity(tmp_path):
    p=write_registration(tmp_path);e=load_registration(p,ROOT/'maps/hall_01.yaml')
    assert e['map_to_world']==[.2,-.1,.3]
    assert registration_from_metadata(dict(registration=e),ROOT/'maps/hall_01.yaml')==e
    p=write_registration(tmp_path,[0,0,0]);assert load_registration(p,ROOT/'maps/hall_01.yaml')['map_to_world']==[0.,0.,0.]
    assert load_registration(None,ROOT/'maps/hall_01.yaml') is None
    assert registration_from_metadata(dict(map_to_world=[0,0,0]),None) is None
    with pytest.raises(ValueError,match='Thiếu'):load_registration(None,ROOT/'maps/hall_01.yaml',required=True)


@pytest.mark.parametrize('change',['nonfinite','source','map','world','verification','convention'])
def test_invalid_registration_is_not_silently_identity(tmp_path,change):
    p=write_registration(tmp_path);d=json.loads(p.read_text())
    if change=='nonfinite':d['map_to_world'][0]=float('nan')
    elif change=='source':d['source']['independent_of_amcl']=False
    elif change=='map':d['assets']['map_yaml_sha256']='wrong'
    elif change=='world':d['assets']['world_sdf_sha256']='wrong'
    elif change=='verification':d['verification']['status']='not_verified'
    else:d['convention']='world_to_map'
    p.write_text(json.dumps(d))
    with pytest.raises(ValueError):load_registration(p,ROOT/'maps/hall_01.yaml')


def test_registration_reaches_runner_spec_and_audit_tuning(tmp_path):
    import numpy as np
    from rl_training.tests.test_ablation_suite import args
    from rl_training.ablation_suite import make_plan,group_spec
    from rl_training.tests.test_fixed3_metrics import rows,REF
    from rl_training.localization_math import compose
    from rl_training.audit_mission_measurements import audit_episode
    from rl_training.tune_pid import measure
    from rl_training.calibrate_pid_measured import measurement
    p=write_registration(tmp_path);e=load_registration(p,ROOT/'maps/hall_01.yaml')
    plan=make_plan(args(registration=str(p)));spec=group_spec(plan,plan['groups'][0],tmp_path/'unused')
    assert spec['registration']==e
    assert registration_from_metadata(spec,spec['map'])['map_to_world']==e['map_to_world']
    folder=tmp_path/'PID';folder.mkdir();session=folder/'episode_00001';session.mkdir()
    (tmp_path/'reference_path.json').write_text(json.dumps(REF));meta=dict(map=str(ROOT/'maps/hall_01.yaml'),registration=e)
    (folder/'metadata.json').write_text(json.dumps(meta));data=rows()
    for r in data:
        v=compose(e['map_to_world'],[r['amcl_x'],r['amcl_y'],r['amcl_yaw']])
        r.update(ground_truth_x=v[0],ground_truth_y=v[1],ground_truth_yaw=v[2],roll=0.,pitch=0.)
    data[-1]['outcome']='SUCCEEDED'
    (session/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in data))
    result,detail=audit_episode(session,REF,e['map_to_world'])
    assert result['full_pass'] is True
    assert detail['amcl_gt_comparison']['rmse_xy_m']<1e-12
    assert measure(tmp_path,'PID')['metrics']['mission_success_pct']['mean']==100.
    assert measurement(folder)['episodes'][0]['full_pass'] is True
    meta.pop('registration');(folder/'metadata.json').write_text(json.dumps(meta))
    with pytest.raises(ValueError,match='thiếu'):measure(tmp_path,'PID')
    with pytest.raises(ValueError,match='thiếu'):measurement(folder)
