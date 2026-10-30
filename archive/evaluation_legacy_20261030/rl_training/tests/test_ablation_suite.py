"""Planning/validation/resume tests without launching Gazebo or unpickling models."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import zipfile

import pytest
import yaml

from rl_training import ablation_suite as suite


def args(**overrides):
    data = dict(modes=['PI'], checkpoints=None, episodes=2, seed=1, gui=False,
                config=str(suite.ROOT/'rl_training/config/sac.yaml'), pid_config=None,
                map=str(suite.ROOT/'maps/hall_01.yaml'),
                nav2_config=str(suite.ROOT/'src/nino_navigation/config/nav2.yaml'),
                reference_path=None, device='cpu', max_episode_seconds=None)
    data.update(overrides)
    return argparse.Namespace(**data)


def model(tmp_path, steps=1234):
    path = tmp_path/'misleading_70000.zip'
    with zipfile.ZipFile(path, 'w') as archive:
        archive.writestr('data', json.dumps(dict(num_timesteps=steps, ent_coef='auto', target_entropy='auto')))
        archive.writestr('policy.pth', b'not-unpickled-by-planner')
        archive.writestr('pytorch_variables.pth', b'not-unpickled-by-planner')
    (tmp_path/'config.yaml').write_text((suite.ROOT/'rl_training/config/sac.yaml').read_text())
    (tmp_path/'metadata.json').write_text(json.dumps(dict(observations=suite.OBSERVATIONS,
        action_order=['left', 'right'], action_scale_nm=0.3)))
    return str(path)


def test_checkpoint_uses_saved_counter_not_name(tmp_path):
    path = model(tmp_path)
    plan = suite.make_plan(args(modes=['PI', 'SAC'], checkpoints=[path]))
    assert plan['groups'][1]['checkpoint']['timesteps'] == 1234
    assert '1234steps' in plan['groups'][1]['label']
    assert len(plan['reference']['poses']) == 221
    assert plan['groups'][0]['gains']['velocity_kd'] == 0
    spec = suite.group_spec(plan, plan['groups'][0], tmp_path/'out')
    assert spec['config']['environment']['benchmark_controller']['velocity_kd'] == 0
    assert 'use_sim_time' not in spec['config']['environment']['benchmark_controller']
    assert plan['config']['environment'].get('benchmark_controller') is None
    with pytest.raises(ValueError, match='Trùng nội dung'):
        suite.make_plan(args(modes=['SAC'], checkpoints=[path, path]))


def test_pid_must_be_explicit_and_does_not_change_sac(tmp_path):
    with pytest.raises(ValueError, match='chưa có PID'):
        suite.make_plan(args(modes=['PI', 'PID']))
    config = tmp_path/'pid.yaml'
    config.write_text(yaml.safe_dump(dict(velocity_kp=0.25, velocity_ki=0.08,
                                          velocity_kd=0.01, derivative_filter_seconds=0.08)))
    # Các gain này là giá trị kiểm thử phần mềm, không phải kết quả tuning.
    plan = suite.make_plan(args(modes=['PID', 'PI'], pid_config=str(config)))
    assert plan['groups'][0]['gains']['velocity_kp'] == 0.25
    assert plan['groups'][1]['gains']['velocity_kp'] == 0.30
    assert plan['controller']['velocity_kd'] == 0
    config.write_text('velocity_kp: 0.3\nvelocity_ki: 0.1\nvelocity_kd: 0\nderivative_filter_seconds: 0.05\n')
    with pytest.raises(ValueError, match='Kd>0'):
        suite.read_pid(config)


def test_reject_unsupported_checkpoint_and_wrong_scale(tmp_path):
    path = Path(model(tmp_path))
    wrong = tmp_path/'model.pt'
    wrong.write_bytes(path.read_bytes())
    with pytest.raises(ValueError, match='chỉ hỗ trợ'):
        suite.make_plan(args(modes=['SAC'], checkpoints=[str(wrong)]))
    (tmp_path/'metadata.json').write_text(json.dumps(dict(observations=suite.OBSERVATIONS,
        action_order=['left', 'right'], action_scale_nm=0.2)))
    with pytest.raises(ValueError, match='Residual scale'):
        suite.make_plan(args(modes=['SAC'], checkpoints=[str(path)]))


def test_saved_reference_endpoint_and_shape_validation(tmp_path):
    config = yaml.safe_load(Path(args().config).read_text())
    data = suite.reference_data(None, config)
    ref = tmp_path/'ref.json'
    ref.write_text(json.dumps(data))
    assert suite.geometry_hash(suite.reference_data(ref, config)) == suite.geometry_hash(data)
    data['poses'][-1]['x'] = 4.0
    ref.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='endpoint'):
        suite.reference_data(ref, config)


def test_serial_execution_stop_resume_and_no_overwrite(tmp_path, monkeypatch):
    from rl_training import benchmark_suite
    plan = suite.make_plan(args())
    second = deepcopy(plan['groups'][0])
    second['label'] = 'second_test_group'
    plan['groups'].append(second)
    calls = []
    reports = []
    fail = [True]
    def worker(spec_path, log):
        spec = json.loads(spec_path.read_text())
        calls.append(spec['group']['label'])
        folder = Path(spec['run'])
        if fail[0]:
            fail[0] = False
            suite.save_json(folder/'result.json', [dict(outcome='FAILED')])
            return 130
        assert len(calls) == 2 or json.loads((tmp_path/'out/PI_only/result.json').read_text())[-1]['outcome'] == 'SUCCEEDED'
        suite.save_json(folder/'result.json', [dict(outcome='SUCCEEDED')]*2)
        return 0
    monkeypatch.setattr(suite, 'launch_worker', worker)
    monkeypatch.setattr(suite, 'active_gazebo', lambda: [])
    monkeypatch.setattr(benchmark_suite, 'run', lambda *a: reports.append(a))
    output = tmp_path/'out'
    assert suite.execute(plan, output) == 130
    assert calls == ['PI_only'] and not reports
    assert suite.execute(plan, output, resume=True) == 0
    assert calls == ['PI_only', 'PI_only', 'second_test_group']
    assert len(reports) == 1
    assert suite.execute(plan, output, resume=True) == 0
    assert len(calls) == 3
    with pytest.raises(FileExistsError):
        suite.execute(plan, output)
    plan['seed'] = 7
    with pytest.raises(ValueError, match='cùng CLI'):
        suite.execute(plan, output, resume=True)


def test_resume_recovers_terminal_before_result_write(tmp_path):
    folder = tmp_path/'episode_00001_test'
    folder.mkdir()
    rows = [dict(outcome=None, seed_label=3, reference_geometry_sha256='same', reward_terms={'a': 1}),
            dict(outcome='FAILED', seed_label=3, reference_geometry_sha256='same', reward_terms={'a': -2})]
    telemetry = folder/'steps.jsonl'
    telemetry.write_text('\n'.join(json.dumps(r) for r in rows))
    spec = dict(seed=3, episodes=7, reference_geometry_sha256='same')
    results = suite.completed_results(tmp_path, spec)
    assert len(results) == 1 and results[0]['reward'] == -1
    assert results[0]['outcome'] == 'FAILED'
    assert suite.completed_results(tmp_path, spec) == results


def test_gazebo_detection_includes_rewritten_process_title(tmp_path, monkeypatch):
    proc = tmp_path/'proc'
    proc.mkdir()
    for pid, title, state in [(11, 'gz sim server', 'S'), (12, 'gz sim gui', 'Z'),
                              (13, '/usr/bin/python3', 'S')]:
        folder = proc/str(pid)
        folder.mkdir()
        (folder/'cmdline').write_bytes(title.encode()+b'\0\0')
        (folder/'stat').write_text(f'{pid} ({title}) {state} 1 0 0')
    original_path = Path
    monkeypatch.setattr(suite, 'Path', lambda value: proc if value == '/proc' else original_path(value))
    assert suite.active_gazebo() == [11]
