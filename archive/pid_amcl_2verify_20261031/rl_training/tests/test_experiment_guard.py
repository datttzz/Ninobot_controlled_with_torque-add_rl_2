from copy import deepcopy
from pathlib import Path
import pytest
from rl_training import experiment_guard as guard
from rl_training.tune_pid import eligibility


@pytest.mark.parametrize('values,match',[
    (dict(memory_available_gb=1.9,disk_free_gb=10,max_temperature_c=60),'RAM'),
    (dict(memory_available_gb=10,disk_free_gb=2.9,max_temperature_c=60),'disk')])
def test_resource_thresholds_stop(values,match,tmp_path,monkeypatch):
    monkeypatch.setattr(guard,'resource_snapshot',lambda _:values)
    with pytest.raises(guard.ResourceStop,match=match):guard.ResourceGuard(tmp_path).check(force=True)


def test_temperature_requires_three_checks_and_deadline_is_absolute(tmp_path,monkeypatch):
    monkeypatch.setattr(guard,'resource_snapshot',lambda _:dict(memory_available_gb=10,disk_free_gb=10,max_temperature_c=96))
    resource=guard.ResourceGuard(tmp_path)
    resource.check(force=True);resource.check(force=True)
    with pytest.raises(guard.ResourceStop,match='Temperature'):resource.check(force=True)
    monkeypatch.setattr(guard,'monotonic',lambda:100.)
    with pytest.raises(guard.ResourceStop,match='budget'):guard.ResourceGuard(tmp_path,dict(deadline_monotonic=99)).check()


def test_active_job_detection_excludes_parent_and_matches_script_args(tmp_path,monkeypatch):
    proc=tmp_path/'proc';proc.mkdir()
    for pid,args in [(11,['python','rl_training/train.py']),(12,['python','rl_training/tune_pid.py']),
                     (13,['bash','-c','text that mentions train.py']),(14,['python','other.py'])]:
        folder=proc/str(pid);folder.mkdir();(folder/'cmdline').write_bytes('\0'.join(args).encode())
    monkeypatch.setattr(guard,'ancestors',lambda:{12})
    monkeypatch.setattr(guard,'Path',lambda value:proc if value=='/proc' else Path(value))
    assert [job['pid'] for job in guard.active_jobs()]==[11]


def measurement(**values):
    metrics=dict(mission_success_pct=100,nav_success_pct=100,amcl_rmse_m=.04,
                 slew_limit_pct=20,saturation_pct=.05,wheel_rmse_rad_s=1.,torque_peak_nm=1.)
    metrics.update(values)
    return dict(episodes=2,metrics={key:dict(mean=value,p95=value) for key,value in metrics.items()})


def test_selection_requires_all_predeclared_constraints():
    baseline=measurement()
    candidate=measurement(wheel_rmse_rad_s=.9,amcl_rmse_m=.043,
                          slew_limit_pct=20.5,saturation_pct=.1)
    assert eligibility(candidate,baseline)==[]
    candidate['metrics']['slew_limit_pct']['mean']=22
    assert any('slew_limit' in reason for reason in eligibility(candidate,baseline))
    assert any('success' in reason for reason in eligibility(measurement(mission_success_pct=50),baseline))
