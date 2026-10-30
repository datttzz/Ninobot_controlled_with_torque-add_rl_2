"""Chạy tuần tự PI/PID/SAC, lưu bằng chứng và gọi benchmark offline."""
import argparse
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import zipfile

import yaml

from .experiment_guard import active_jobs, ResourceGuard, ResourceStop

ROOT = Path(__file__).resolve().parents[1]
PI_GAINS = dict(velocity_kp=0.30, velocity_ki=0.10, velocity_kd=0.0,
                derivative_filter_seconds=0.05)
OBSERVATIONS = ['cross_track', 'heading_error', 'remaining', 'roll', 'pitch',
                'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z',
                'speed', 'yaw_rate', 'cmd_speed', 'cmd_yaw_rate', 'wheel_left', 'wheel_right',
                'base_left', 'base_right', 'previous_action_left', 'previous_action_right',
                'residual_left', 'residual_right']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temp.replace(path)


def resolved(value):
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'Không tìm thấy file: {path}')
    return path


def read_pid(path):
    if path is None:
        raise ValueError('Cần file cấu hình PID')
    data = yaml.safe_load(resolved(path).read_text())
    required = set(PI_GAINS)
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError('PID config chỉ chứa: '+', '.join(sorted(required)))
    for name, value in data.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f'Gain PID chưa hợp lệ: {name}={value}')
    if any(data[k] <= 0 for k in ('velocity_kp','velocity_ki','velocity_kd')) or data['derivative_filter_seconds'] <= 0:
        raise ValueError('PID cần Kp,Ki,Kd>0; tau_D>0')
    return {key: float(value) for key, value in data.items()}


def checkpoint_info(path, common):
    path = resolved(path)
    if path.suffix.lower() != '.zip':
        raise ValueError(f'{path.name}: chỉ hỗ trợ SAC Stable-Baselines3 .zip. '
                         '.pt/.pth không chứa đủ kiến trúc/normalization của môi trường này; '
                         'không đổi đuôi file để giả làm checkpoint.')
    try:
        with zipfile.ZipFile(path) as archive:
            data = json.loads(archive.read('data'))
            if 'ent_coef' not in data or 'target_entropy' not in data:
                raise ValueError('Archive không phải SAC')
            for name in ('policy.pth', 'pytorch_variables.pth'):
                if name not in archive.namelist():
                    raise ValueError('Thiếu '+name)
            steps = data['num_timesteps']
            if isinstance(steps, bool) or not isinstance(steps, int) or steps < 0:
                raise ValueError('num_timesteps không hợp lệ')
    except (zipfile.BadZipFile, KeyError, json.JSONDecodeError) as exc:
        raise ValueError(f'Checkpoint SB3 không hợp lệ: {path}: {exc}') from exc
    folder = path.parent.parent if path.parent.name == 'checkpoints' else path.parent
    config_path, metadata_path = folder/'config.yaml', folder/'metadata.json'
    if not config_path.is_file() or not metadata_path.is_file():
        raise ValueError(f'Cần config.yaml và metadata.json nguồn bên cạnh {path}')
    config = yaml.safe_load(config_path.read_text())
    metadata = json.loads(metadata_path.read_text())
    if metadata.get('observations') != OBSERVATIONS or metadata.get('action_order') != ['left', 'right']:
        raise ValueError(f'Checkpoint dùng observation/action khác repo: {path}')
    # Không thay đổi scale của policy khi chuyển checkpoint.
    if metadata.get('action_scale_nm') != common['environment']['residual_limit']:
        raise ValueError(f'Residual scale khác cấu hình chung: {path}')
    for key in ('residual_limit', 'max_path_deviation'):
        if config['environment'][key] != common['environment'][key]:
            raise ValueError(f'Observation normalization khác ở {key}: {path}')
    return dict(path=str(path), sha256=sha(path), timesteps=steps,
                config=str(config_path), config_sha256=sha(config_path),
                metadata=str(metadata_path), metadata_sha256=sha(metadata_path),
                training_environment=config['environment'], training_reward=config['reward'],
                pi_gain_provenance='Legacy metadata has no gain snapshot; repo PI 0.30/0.10 assumed')


def reference_data(path, config):
    goal_x, goal_y = (float(config['environment'][k]) for k in ('goal_x', 'goal_y'))
    if path:
        data = json.loads(resolved(path).read_text())
    else:
        # Đường tâm hành lang mặc định; cùng hình học, không gọi planner mỗi episode.
        count = max(1, math.ceil(math.hypot(goal_x, goal_y)/0.05))
        yaw = math.atan2(goal_y, goal_x)
        data = dict(frame_id='map', poses=[dict(x=goal_x*i/count, y=goal_y*i/count, yaw=yaw)
                                         for i in range(count+1)])
        # NavigateToPose của môi trường hiện dùng hướng đích bằng 0.
        data['poses'][-1]['yaw'] = 0.0
    if data.get('frame_id') != 'map' or len(data.get('poses', [])) < 2:
        raise ValueError('Reference JSON cần frame_id=map và poses >=2')
    poses = []
    for row in data['poses']:
        if any(isinstance(row.get(k), bool) or not isinstance(row.get(k), (int, float))
               or not math.isfinite(row[k]) for k in ('x', 'y', 'yaw')):
            raise ValueError('Reference cần x,y,yaw hữu hạn')
        poses.append({k: float(row[k]) for k in ('x', 'y', 'yaw')})
    if math.hypot(poses[0]['x'], poses[0]['y']) > 1e-6:
        raise ValueError('Spawn cố định tại (0,0); reference phải bắt đầu tại đó')
    end = poses[-1]
    if math.hypot(end['x']-goal_x, end['y']-goal_y) > 1e-6 or abs(math.remainder(end['yaw'], 2*math.pi)) > 1e-6:
        raise ValueError('Reference endpoint phải trùng goal_x/goal_y và yaw=0')
    if any(math.hypot(a['x']-b['x'], a['y']-b['y']) < 1e-9 for a, b in zip(poses, poses[1:])):
        raise ValueError('Reference chứa hai điểm liên tiếp trùng nhau')
    return dict(frame_id='map', poses=poses)


def geometry_hash(reference):
    geometry = ['map', [['map', p['x'], p['y'], 0.0, 0.0, 0.0,
                         math.sin(p['yaw']/2), math.cos(p['yaw']/2)] for p in reference['poses']]]
    return hashlib.sha256(json.dumps(geometry, separators=(',', ':')).encode()).hexdigest()


def make_plan(args):
    config = yaml.safe_load(resolved(args.config).read_text())
    purpose = getattr(args, 'purpose', 'evaluation')
    if purpose == 'smoke':
        config['environment']['max_episode_seconds'] = 3.0
    if args.max_episode_seconds is not None:
        config['environment']['max_episode_seconds'] = args.max_episode_seconds
    config['environment']['headless'] = not args.gui
    config['environment']['resource_guard'] = {}
    if getattr(args,'deadline_monotonic',None):
        config['environment']['resource_guard']['deadline_monotonic'] = args.deadline_monotonic
    if args.episodes < 1 or args.seed < 0:
        raise ValueError('episodes phải >0; seed phải >=0')
    for key in ('max_episode_seconds', 'step_seconds', 'max_path_deviation', 'residual_limit'):
        value = config['environment'][key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Cấu hình environment không hợp lệ: '+key)
    if len(args.modes) != len(set(args.modes)):
        raise ValueError('Không lặp mode')
    if bool(args.sac_checkpoint) != ('SAC' in args.modes):
        raise ValueError('Chọn SAC cần đúng một --sac-checkpoint')
    pid = read_pid(args.pid_config) if 'PID' in args.modes else None
    control_path = ROOT/'src/nino_control/config/effort_drive.yaml'
    control = yaml.safe_load(control_path.read_text())['effort_drive']['ros__parameters']
    # PI nền từng dùng khi train; không đổi gain nguồn hay giới hạn chấp hành.
    for key in ('velocity_kp', 'velocity_ki'):
        if control[key] != PI_GAINS[key]:
            raise ValueError('PI repo đã đổi so với gain nền checkpoint: '+key)
    control.update(PI_GAINS)
    groups = []
    for mode in args.modes:
        if mode == 'SAC':
            evidence = checkpoint_info(args.sac_checkpoint, config)
            groups.append(dict(label='PI_SAC', mode=mode, gains=PI_GAINS, checkpoint=evidence))
        else:
            groups.append(dict(label=mode, mode=mode,
                               gains=pid if mode == 'PID' else PI_GAINS, checkpoint=None))
    map_path = resolved(args.map)
    map_data = yaml.safe_load(map_path.read_text())
    map_image = resolved(map_path.parent/Path(map_data['image']).expanduser())
    reference = reference_data(args.reference_path, config)
    sources = [map_path, map_image, resolved(args.nav2_config), control_path,
               ROOT/'src/nino_description/worlds/long_hall.sdf',
               ROOT/'src/nino_control/nino_control/effort_drive.py',
               ROOT/'src/nino_control/nino_control/derivative.py',
               ROOT/'src/nino_navigation/scripts/episode_manager.py',
               ROOT/'rl_training/ros_env.py', ROOT/'rl_training/ablation_suite.py']
    sources += list((ROOT/'src/nino_description/urdf').rglob('*.xacro'))
    sources += [ROOT/p for p in ('src/nino_description/launch/sim.launch.py',
                                 'src/nino_description/scripts/gz_sim_clean',
                                 'src/nino_navigation/launch/navigation.launch.py',
                                 'src/nino_navigation/launch/fixed_path.launch.py',
                                 'src/nino_navigation/config/episode.yaml',
                                 'rl_training/experiment_guard.py')]
    return dict(version=2, purpose=purpose, episodes=args.episodes, seed=args.seed, device=args.device,
                config=config, controller=control, groups=groups,
                map=str(map_path), nav2_config=str(resolved(args.nav2_config)),
                reference=reference, reference_geometry_sha256=geometry_hash(reference),
                sources={str(p): sha(p) for p in sources},
                ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                gz_partition=os.environ.get('GZ_PARTITION'),
                discovery_range=os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE'),
                caveats=['Seeds only label Gym resets, not Gazebo/AMCL physical noise.',
                         'Same spawn/map/reference/Nav2/limits; localization and physics can vary.',
                         'Default reference is a map-frame centreline, not a collision-free planner certificate.',
                         'Legacy SAC metadata has no historical wheel-gain/config hash.',
                         'Benchmark sensors logged at 50 Hz, latest samples with delivery times; no interpolation across missing data.'])


def group_spec(plan, group, output):
    folder = output/group['label']
    config = deepcopy(plan['config'])
    controller = deepcopy(plan['controller'])
    controller.update(group['gains'])
    config['environment'].update(effort_drive_config=str(folder/'controller.yaml'),
                                 nav2_params_file=str(output/'nav2.yaml'),
                                 reference_path_file=str(output/'reference_path.json'),
                                 benchmark_controller={k: float(v) for k, v in controller.items()
                                                       if isinstance(v, (int, float)) and not isinstance(v, bool)})
    return dict(group=group, run=str(folder), config=config, map=plan['map'],
                purpose=plan['purpose'],
                episodes=plan['episodes'], seed=plan['seed'], device=plan['device'],
                nav2_sha256=sha(plan['nav2_config']),
                reference_geometry_sha256=plan['reference_geometry_sha256'],
                sources=plan['sources'], controller=controller)


def completed_results(folder, spec):
    """Khôi phục cửa sổ Ctrl+C giữa ghi terminal sample và ghi result.json."""
    path = folder/'result.json'
    results = json.loads(path.read_text()) if path.exists() else []
    sessions = {r['session'] for r in results}
    indices = {r['episode'] for r in results}
    for telemetry in sorted(folder.glob('episode_*/steps.jsonl')):
        session = str(telemetry.parent)
        if session in sessions:
            continue
        try:
            rows = [json.loads(line) for line in telemetry.read_text().splitlines() if line.strip()]
        except json.JSONDecodeError:
            continue  # File bị ngắt vẫn giữ để benchmark audit.
        if not rows or rows[-1].get('outcome') not in ('SUCCEEDED', 'FAILED', 'TIME_LIMIT'):
            continue
        last = rows[-1]
        index = last.get('seed_label')
        if not isinstance(index, int):
            raise ValueError('Terminal telemetry thiếu seed_label: '+str(telemetry))
        index -= spec['seed']
        if index in indices or not 0 <= index < spec['episodes']:
            raise ValueError('Duplicate/out-of-range terminal episode: '+str(telemetry))
        if any(r.get('reference_geometry_sha256') != spec['reference_geometry_sha256'] for r in rows):
            raise ValueError('Recovered episode reference mismatch')
        results.append(dict(episode=index, seed=last['seed_label'],
                            reward=sum(sum(r['reward_terms'].values()) for r in rows),
                            steps=len(rows), outcome=last['outcome'], session=session,
                            reference_geometry_sha256=last['reference_geometry_sha256']))
        indices.add(index)
    results.sort(key=lambda r: r['episode'])
    if [r['episode'] for r in results] != list(range(len(results))):
        raise ValueError('Completed episode indices are not contiguous')
    if results:
        save_json(path, results)
    return results


def worker(spec_path):
    # Import nặng chỉ xảy ra trong tiến trình đánh giá; --help/--dry-run không cần ROS.
    sys.path.insert(0, str(ROOT/'rl_training'))
    from plotting_compat import prepare_matplotlib_namespace
    prepare_matplotlib_namespace()
    import numpy as np
    import torch
    from stable_baselines3 import SAC
    from ros_env import NinoResidualEnv
    from runtime import resolve_device
    from episode_outcome import InfrastructureFault
    from startup import StartupFault
    spec = json.loads(Path(spec_path).read_text())
    folder = Path(spec['run'])
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    for path, digest in spec['sources'].items():
        if sha(path) != digest:
            raise ValueError('Source changed since planning: '+path)
    if sha(spec['config']['environment']['nav2_params_file']) != spec['nav2_sha256']:
        raise ValueError('Snapshot Nav2 changed')
    reference = json.loads(Path(spec['config']['environment']['reference_path_file']).read_text())
    if geometry_hash(reference) != spec['reference_geometry_sha256']:
        raise ValueError('Snapshot reference changed')
    parameters = yaml.safe_load(Path(spec['config']['environment']['effort_drive_config']).read_text())
    if parameters != {'effort_drive': {'ros__parameters': spec['controller']}}:
        raise ValueError('Snapshot controller changed')
    group, model, env = spec['group'], None, None
    results_path = folder/'result.json'
    results = completed_results(folder, spec)
    try:
        torch.set_num_threads(1)
        device, description = resolve_device(spec['device'] if group['mode'] == 'SAC' else 'cpu')
        if group['checkpoint']:
            checkpoint = group['checkpoint']
            if sha(checkpoint['path']) != checkpoint['sha256']:
                raise ValueError('Checkpoint changed after planning')
            model = SAC.load(checkpoint['path'], device=device)
            if model.num_timesteps != checkpoint['timesteps']:
                raise ValueError('Loaded timestep count differs from archive')
            if model.observation_space.shape != (23,) or model.action_space.shape != (2,):
                raise ValueError('Checkpoint spaces differ from Nino env')
            if not np.allclose(model.action_space.low, -1) or not np.allclose(model.action_space.high, 1):
                raise ValueError('Checkpoint action bounds mismatch')
        calibration_path=ROOT/'rl_training/config/PID_CALIBRATION.json'
        calibration=json.loads(calibration_path.read_text()) if calibration_path.exists() else {}
        selected=calibration.get('selected') or {}
        pid_evidence=dict(record=str(calibration_path) if calibration else None,
            selected_profile_matches=bool(selected) and all(selected.get(k)==v for k,v in group['gains'].items()),
            tuning_dir=calibration.get('tuning_dir'),profile_updated=calibration.get('profile_updated',False))
        metadata = dict(pid_calibration=pid_evidence if group['mode']=='PID' else None, map_to_world=[0.0,0.0,0.0], mode='evaluate' if model is not None else 'baseline',
                        experiment_purpose=spec['purpose'],
                        controller_mode=group['mode'], seed=spec['seed'], map=spec['map'],
                        deterministic=True, observations=OBSERVATIONS,
                        action_order=['left', 'right'], action_scale_nm=spec['config']['environment']['residual_limit'],
                        model=group['checkpoint']['path'] if model is not None else None,
                        checkpoint_evidence=group['checkpoint'], gains=group['gains'],
                        nav2_snapshot_sha256=spec['nav2_sha256'],
                        config_snapshot_sha256=sha(folder/'config.yaml'),
                        controller_snapshot_sha256=sha(folder/'controller.yaml'),
                        reference_geometry_sha256=spec['reference_geometry_sha256'],
                        telemetry_schema='nino-benchmark-v2-50Hz',
                        acceleration_convention='body acceleration minus quaternion gravity 9.80665 m/s2',
                        sources=spec['sources'], controller=spec['controller'],
                        pose_source='AMCL map->base_footprint; ground_truth_* separately in Gazebo frame',
                        seed_scope='Gym only; not Gazebo/AMCL noise', device=device, device_info=description)
        save_json(folder/'metadata.json', metadata)
        env = NinoResidualEnv(spec['config'], spec['map'], folder, residual_enabled=model is not None)
        for index in range(len(results), spec['episodes']):
            observation, info = env.reset(seed=spec['seed']+index)
            if info['reference_geometry_sha256'] != spec['reference_geometry_sha256']:
                raise InfrastructureFault('Reference geometry mismatch; refusing unequal conditions')
            total, steps = 0.0, 0
            while True:
                action = np.zeros(2, dtype=np.float32) if model is None else model.predict(observation, deterministic=True)[0]
                observation, reward, terminated, truncated, info = env.step(action)
                if info['reference_geometry_sha256'] != spec['reference_geometry_sha256']:
                    raise InfrastructureFault('Reference changed during episode')
                if model is None and any(abs(v) > 1e-12 for v in info['residual_applied']):
                    raise InfrastructureFault('Residual was applied in a pure baseline')
                total += reward
                steps += 1
                if terminated or truncated:
                    break
            results.append(dict(episode=index, seed=spec['seed']+index, reward=total,
                                steps=steps, outcome=info['outcome'],
                                reference_geometry_sha256=info['reference_geometry_sha256'],
                                session=str(env.session_dir)))
            save_json(results_path, results)
            print(json.dumps(results[-1]), flush=True)
        return 0
    except KeyboardInterrupt:
        with (folder/'attempt_events.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(event='interrupted', completed=len(results), utc=datetime.now(timezone.utc).isoformat()))+'\n')
        return 130
    except Exception as exc:
        kind = 'infrastructure_fault' if isinstance(exc, (InfrastructureFault, StartupFault, ResourceStop)) else 'exception'
        event = dict(event=kind, reason=str(exc), type=type(exc).__name__, completed=len(results),
                     session=str(env.session_dir) if env is not None and hasattr(env, 'session_dir') else None,
                     utc=datetime.now(timezone.utc).isoformat())
        save_json(folder/(kind+'.json'), event)
        with (folder/'attempt_events.jsonl').open('a') as stream:
            stream.write(json.dumps(event)+'\n')
        print(json.dumps(event), file=sys.stderr, flush=True)
        return 2
    finally:
        if env is not None:
            env.close()


def launch_worker(spec, log):
    # Chỉ một worker đang chạy. SIGINT được chuyển cho worker để nó cleanup
    # toàn bộ session do nó sở hữu, không pkill tiến trình bên ngoài.
    with log.open('a') as stream:
        child = subprocess.Popen([sys.executable, str(ROOT/'run_ablation_suite.py'),
                                  '--worker-spec', str(spec)], stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            return child.wait()
        except KeyboardInterrupt:
            child.send_signal(signal.SIGINT)
            print('Đang dừng worker và Gazebo/Nav2 do suite tạo...', flush=True)
            while child.poll() is None:
                try:
                    child.wait(timeout=1)
                except (subprocess.TimeoutExpired, KeyboardInterrupt):
                    continue
            return 130


def active_gazebo():
    pids = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            args = (process/'cmdline').read_bytes().decode(errors='replace').split('\0')
            names = [Path(a).name for a in args[:2]]
            title = args[0].strip() if args else ''
            if (title in ('gz sim server', 'gz sim gui')
                    or any('gz-sim' in a for a in names) or ('sim' in args and 'gz' in names)):
                state = (process/'stat').read_text().split(') ', 1)[1].split()[0]
                if state == 'Z':
                    continue
                pids.append(int(process.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return pids


@contextmanager
def machine_lock():
    import fcntl
    # Chung cho mọi output-root của cùng tài khoản; không chạy song song hai suite.
    with Path(f'/tmp/nino_ablation_{os.getuid()}.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('Một ablation suite khác đang chạy trên máy') from exc
        yield


def execute(plan, output, resume=False):
    import fcntl
    output.mkdir(parents=True, exist_ok=resume)
    with (output/'.suite.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('Một runner khác đang dùng output này') from exc
        if resume:
            if json.loads((output/'manifest.json').read_text()) != plan:
                raise ValueError('--resume cần cùng CLI, nguồn code/config/checkpoint và ROS/GZ environment')
        else:
            save_json(output/'manifest.json', plan)
            save_json(output/'reference_path.json', plan['reference'])
            (output/'nav2.yaml').write_text(Path(plan['nav2_config']).read_text())
        state = {}
        for group in plan['groups']:
            spec = group_spec(plan, group, output)
            folder = Path(spec['run'])
            folder.mkdir(exist_ok=True)
            if not (folder/'worker_spec.json').exists():
                (folder/'config.yaml').write_text(yaml.safe_dump(spec['config'], sort_keys=False))
                (folder/'controller.yaml').write_text(yaml.safe_dump(
                    {'effort_drive': {'ros__parameters': spec['controller']}}, sort_keys=False))
                save_json(folder/'worker_spec.json', spec)
            else:
                if json.loads((folder/'worker_spec.json').read_text()) != spec:
                    raise ValueError('Worker spec changed: '+str(folder))
            result_path = folder/'result.json'
            done = len(json.loads(result_path.read_text())) if result_path.exists() else 0
            if done >= plan['episodes']:
                state[group['label']] = dict(status='complete', episodes=done)
                continue
            pids = active_gazebo()
            if pids:
                raise ValueError(f'Gazebo vẫn đang chạy (PID {pids}); hãy dừng phiên đó trước. Suite không tự kill phiên ngoài.')
            print(f'Chạy {group["label"]}: {done}/{plan["episodes"]}; log: {folder/"runner.log"}', flush=True)
            code = launch_worker(folder/'worker_spec.json', folder/'runner.log')
            if code != 0:
                with (folder/'attempt_events.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(event='worker_exit',exit_code=code,
                        reason='See runner.log; no controller failure inferred',
                        utc=datetime.now(timezone.utc).isoformat()))+'\n')
            lingering = active_gazebo()
            if lingering:
                event = dict(event='infrastructure_fault',
                             reason=f'Gazebo remains after worker cleanup: {lingering}',
                             utc=datetime.now(timezone.utc).isoformat())
                with (folder/'attempt_events.jsonl').open('a') as stream:
                    stream.write(json.dumps(event)+'\n')
                code = 2
            done = len(json.loads(result_path.read_text())) if result_path.exists() else 0
            state[group['label']] = dict(status='complete' if code == 0 else 'interrupted' if code == 130 else 'error',
                                         exit_code=code, episodes=done)
            save_json(output/'status.json', state)
            if code != 0:
                print('Suite dừng; giữ nguyên mọi log. Dùng lại CLI với --resume sau khi kiểm tra runner.log.', flush=True)
                return code
        save_json(output/'status.json', state)
        if {g['label'] for g in plan['groups']} == {'PI','PID','PI_SAC'}:
            import benchmark_pi_pid_sac
            report = output/'report'
            if report.exists():
                report = output/('report_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            benchmark_pi_pid_sac.build_report(output/'PI', output/'PID', output/'PI_SAC', report)
            print('Báo cáo:', report/'report_PI_PID_SAC.html', flush=True)
        else:
            print('Run hiệu chỉnh/kiểm tra đã lưu; báo cáo chính thức yêu cầu đủ PI/PID/PI_SAC.', flush=True)
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--modes', nargs='+', choices=['PI', 'PID', 'SAC'], default=['PI', 'PID', 'SAC'])
    parser.add_argument('--sac-checkpoint', help='Một checkpoint SB3 SAC .zip duy nhất')
    parser.add_argument('--episodes', type=int, default=7)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--map', default=str(ROOT/'maps/hall_01.yaml'))
    parser.add_argument('--config', default=str(ROOT/'rl_training/config/sac.yaml'))
    parser.add_argument('--nav2-config', default=str(ROOT/'src/nino_navigation/config/nav2.yaml'))
    parser.add_argument('--pid-config', default=str(ROOT/'rl_training/config/pid.yaml'), help='YAML gain đã hiệu chỉnh riêng; không thay PI nền của SAC')
    parser.add_argument('--reference-path', help='JSON map-frame x/y/yaw; mặc định đường thẳng từ (0,0) tới goal')
    parser.add_argument('--gui', action='store_true', help='Mở Gazebo GUI, vẫn chạy các nhóm tuần tự')
    parser.add_argument('--purpose', choices=['evaluation', 'smoke', 'calibration'], default='evaluation',
                        help='smoke giới hạn 3 giây mô phỏng; chỉ kiểm tra kỹ thuật, không kết luận chất lượng')
    parser.add_argument('--device', choices=['cpu', 'cuda', 'auto'], default='cpu')
    parser.add_argument('--max-episode-seconds', type=float, help='Giới hạn chung; dùng ngắn cho kiểm tra kỹ thuật')
    parser.add_argument('--dry-run', action='store_true', help='Kiểm tra config/checkpoint và in kế hoạch; không mở ROS')
    parser.add_argument('--resume', action='store_true', help='Chạy tiếp các episode chưa hoàn tất trong output-root')
    parser.add_argument('--deadline-monotonic', type=float, help=argparse.SUPPRESS)
    parser.add_argument('--worker-spec', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker_spec:
        return worker(args.worker_spec)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        plan = make_plan(args)
        if args.dry_run:
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            return 0
        if args.output_root is None:
            raise ValueError('Cần --output-root (thư mục mới); thêm --resume nếu chạy tiếp')
        if not os.environ.get('ROS_DISTRO') == 'jazzy':
            raise ValueError('Hãy source /opt/ros/jazzy/setup.bash và install/setup.bash trước')
        output = args.output_root.expanduser().resolve()
        jobs = active_jobs()
        if jobs: raise ValueError(f'Job đang hoạt động: {jobs}; không chạy chồng')
        ResourceGuard(ROOT).check(force=True)
        with machine_lock():
            return execute(plan, output, args.resume)
    except (ValueError, FileNotFoundError, FileExistsError, KeyError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
