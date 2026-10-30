"""Kiểm tra wrapper không bỏ lại Gazebo khi dừng hoặc bị kill đột ngột."""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


WRAPPER = Path(__file__).resolve().parents[2] / 'src/nino_description/scripts/gz_sim_clean'


def alive(pid):
    """Zombie đã kết thúc thực thi, dù init chưa thu hồi ngay."""
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


@pytest.mark.parametrize('signum, ignore', [(signal.SIGINT, False),
                                         (signal.SIGTERM, False),
                                         (signal.SIGKILL, False),
                                         (signal.SIGINT, True)])
def test_owned_gazebo_exits_with_wrapper(tmp_path, signum, ignore):
    """Dùng tiến trình giả để kiểm tra không đụng các phiên Gazebo bên ngoài."""
    fake = tmp_path / 'gz'
    fake.write_text('''#!/usr/bin/python3
import os
from pathlib import Path
import signal
import time
handler = signal.SIG_IGN if os.environ['IGNORE_SIGNALS'] == '1' else lambda *_: exit(0)
signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)
Path(os.environ['PID_FILE']).write_text(str(os.getpid()))
while True:
    time.sleep(0.05)
''')
    fake.chmod(0o755)
    pid_file = tmp_path / 'child.pid'
    env = dict(os.environ, PATH=f'{tmp_path}:/usr/bin:/bin', PID_FILE=str(pid_file),
               IGNORE_SIGNALS='1' if ignore else '0')
    wrapper = subprocess.Popen(['/usr/bin/python3', str(WRAPPER), '-s'], env=env,
                               start_new_session=True, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    child_pid = None
    try:
        deadline = time.monotonic() + 5
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert pid_file.exists(), 'Gazebo giả chưa được tạo'
        child_pid = int(pid_file.read_text())
        assert alive(child_pid)
        wrapper.send_signal(signum)
        wrapper.wait(timeout=5)
        deadline = time.monotonic() + 3
        while alive(child_pid) and time.monotonic() < deadline:
            time.sleep(0.02)
        assert not alive(child_pid), 'Wrapper đã tắt nhưng Gazebo vẫn sống'
    finally:
        if wrapper.poll() is None:
            wrapper.kill()
            wrapper.wait(timeout=5)
        if child_pid is not None and alive(child_pid):
            os.killpg(child_pid, signal.SIGKILL)


@pytest.mark.parametrize('early_exit', [False, True])
def test_gui_server_that_forks_and_changes_session_is_reaped(tmp_path, early_exit):
    """Frontend và GUI server không nhất thiết cùng process group."""
    fake = tmp_path/'gz'
    fake.write_text('''#!/usr/bin/python3
import os, signal, time
from pathlib import Path
pid = os.fork()
if pid == 0:
    os.setsid()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    Path(os.environ['PID_FILE']).write_text(str(os.getpid()))
    while True: time.sleep(.05)
while not Path(os.environ['PID_FILE']).exists(): time.sleep(.01)
if os.environ['EARLY_EXIT'] == '1': os._exit(0)
while True: time.sleep(.05)
''')
    fake.chmod(0o755)
    pid_file = tmp_path/'server.pid'
    env = dict(os.environ, PATH=f'{tmp_path}:/usr/bin:/bin', PID_FILE=str(pid_file),
               EARLY_EXIT=str(int(early_exit)))
    outside = subprocess.Popen(['/usr/bin/python3', '-c', 'import time; time.sleep(30)'])
    wrapper = subprocess.Popen(['/usr/bin/python3', str(WRAPPER)], env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    server = None
    try:
        deadline = time.monotonic()+5
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert pid_file.exists()
        server = int(pid_file.read_text())
        if not early_exit:
            wrapper.send_signal(signal.SIGINT)
        assert wrapper.wait(timeout=6) == 0
        assert not alive(server), 'Server đã setsid bị bỏ lại'
        assert outside.poll() is None, 'Không được tác động tiến trình ngoài'
    finally:
        if wrapper.poll() is None:
            wrapper.kill()
            wrapper.wait(timeout=3)
        if server is not None and alive(server):
            os.kill(server, signal.SIGKILL)
        outside.terminate()
        outside.wait(timeout=3)
