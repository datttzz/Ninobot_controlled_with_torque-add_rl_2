"""Giới hạn tài nguyên và phát hiện job khác; không kill tiến trình ngoài."""
import os
from pathlib import Path
import shutil
from time import monotonic


class ResourceStop(RuntimeError):
    pass


def ancestors():
    result={os.getpid()};pid=os.getpid()
    while pid>1:
        try:pid=int(Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()[1])
        except (OSError,ValueError):break
        result.add(pid)
    return result


def active_jobs():
    excluded=ancestors();result=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded:continue
        try:
            args=(entry/'cmdline').read_bytes().decode(errors='replace').split('\0')
            if any(Path(a).name in ('train.py','long_run.py','run_ablation_suite.py','tune_pid.py','calibrate_pid_measured.py') for a in args[:4]):
                result.append(dict(pid=int(entry.name),command=' '.join(args[:5])))
        except (OSError,ValueError):continue
    return result


def resource_snapshot(directory):
    memory={line.split(':')[0]:int(line.split()[1])*1024 for line in Path('/proc/meminfo').read_text().splitlines()}
    temperatures=[]
    for path in list(Path('/sys/class/thermal').glob('thermal_zone*/temp'))+list(Path('/sys/class/hwmon').glob('hwmon*/temp*_input')):
        try:
            value=float(path.read_text())/1000
            if 0<value<120:temperatures.append(value)
        except (OSError,ValueError):pass
    return dict(memory_available_gb=memory['MemAvailable']/1024**3,
                disk_free_gb=shutil.disk_usage(directory).free/1024**3,
                max_temperature_c=max(temperatures) if temperatures else None)


class ResourceGuard:
    def __init__(self,directory,settings=None):
        self.directory=directory;self.settings=settings or {};self.last_check=-1e9
        self.hot=0;self.last=None

    def check(self,force=False):
        now=monotonic()
        deadline=self.settings.get('deadline_monotonic')
        if deadline is not None and now>=deadline:raise ResourceStop('Wall-clock budget exhausted; stopping owned session')
        if not force and now-self.last_check<2:return self.last
        self.last_check=now;self.last=resource_snapshot(self.directory)
        if self.last['memory_available_gb']<2:raise ResourceStop('Available RAM below 2 GiB')
        if self.last['disk_free_gb']<3:raise ResourceStop('Free disk below 3 GiB')
        temp=self.last['max_temperature_c']
        self.hot=self.hot+1 if temp is not None and temp>=95 else 0
        if self.hot>=3:raise ResourceStop('Temperature >=95 C on three consecutive checks')
        return self.last
