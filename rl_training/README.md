# Residual SAC cho Nino trên laptop Ubuntu

Hướng dẫn chính cho workspace
`/home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2`, chạy trực tiếp
trên Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, RTX 4060 Laptop 8 GB.
Các lệnh dưới đây chạy từ thư mục gốc workspace, không phải bên trong `rl_training/`.

- Mới bắt đầu: xem [hướng dẫn từng terminal trên laptop](../HUONG_DAN_CHAY_TREN_LAPTOP.md).
- Ghi chép máy WSL/Docker và các run cũ: [lịch sử được giữ nguyên](README_WSL_LICH_SU.md).
- Các lệnh trong README này dành cho Ubuntu trực tiếp; không cần `docker exec`,
  PowerShell hoặc đường dẫn container `/ros2_ws`.

Dự án đang ở mức tích hợp/huấn luyện thử. Model học được chưa mặc nhiên chứng
minh cải thiện khả năng vượt gờ; cần baseline và evaluate cùng điều kiện.

## Điều khiển nền và phần RL

Robot trong repo có hai bánh chủ động và hai caster thụ động. RL có hai đầu ra,
theo thứ tự `[trái, phải]`:

```text
Nav2 → vận tốc → PI → mô-men nền ─────────┐
                                        ├→ giới hạn → bánh xe
SAC → action [-1,1]² → nhân 0,3 N·m ─────┘
```

`τ = clip(τ_PI + Δτ_RL, -2, +2)` N·m trong chế độ bám vận tốc hiện tại.
Sau đó lệnh vẫn qua giới hạn độ thay đổi mô-men, bảo vệ tốc độ và giới hạn
phần cứng như trước. Không tăng giới hạn PI hoặc mô-men phần cứng.

`enable_residual` mặc định **false** trong launch mô phỏng. Môi trường RL tự bật
khi train/evaluate và tắt khi baseline. Phần bù chỉ có hiệu lực khi:

- Episode đang `FOLLOWING`.
- Lệnh vận tốc còn mới và khác 0.
- Lệnh RL là hai số hữu hạn, chưa quá 0,2 giây theo đồng hồ thực.
- Không ở chế độ gửi mô-men trực tiếp.

Thất bại/hủy/dừng làm phần bù về 0. Collision monitor và EpisodeManager vẫn giữ
quyền dừng. `/wheel_torque_commands` là lệnh **ghi đè** mô-men, không phải đầu
vào cho RL; RL dùng `/wheel_torque_residual`.

## SAC học gì?

**Observation** là dữ liệu policy nhận vào. Có 23 giá trị được chia tỷ lệ và
giới hạn trong `[-10, 10]`: lỗi ngang và góc so với đường, khoảng đường còn lại,
roll/pitch, gia tốc IMU đã trừ trọng lực, vận tốc góc IMU, vận tốc robot, vận tốc
yêu cầu, tốc độ hai bánh, mô-men PI, action trước và phần bù được chấp nhận.
Thứ tự chính xác ở `ros_env.py: OBSERVATIONS` và được lưu trong metadata mỗi run.

**Action** là hai số trong `[-1, 1]`, tương ứng mô-men bù tối đa ±0,3 N·m.
SAC không thay đích, đường hay lệnh vận tốc Nav2.

**Reward** là điểm để policy học. Trong `features.py`:

```text
r = 20 × tiến triển dọc đường
    - dt × (8 × lỗi ngang² + 0,5 × lỗi góc²
            + 0,02 × ||gia tốc đã trừ trọng lực||²
            + 0,01 × ||action||² + 0,05)
    - 0,01 × ||action - action_trước||²
    + 30 nếu thành công, -30 nếu thất bại hoặc hết thời gian
```

Tiến triển được tính từ vị trí chiếu lên đường trong `map`, không phải chỉ
quãng đường bánh quay. Mức thưởng/phạt này là điểm khởi đầu cần kiểm chứng,
không phải bộ trọng số tối ưu. Bài thực hành cho bạn: xem từng thành phần trong
`steps.jsonl`, giải thích vì sao policy đứng yên không thể nhận thưởng dương.

Mỗi step khoảng 0,1 giây mô phỏng. Phần bù được phát lại trong lúc chờ step.
Nếu việc cập nhật mạng lâu hơn watchdog, hệ thống tự quay về điều khiển nền.
Môi trường theo API [Gymnasium](https://gymnasium.farama.org/introduction/create_custom_env/),
SAC dùng [Stable-Baselines3 2.7.1](https://stable-baselines3.readthedocs.io/en/v2.7.1/modules/sac.html).

## 1. Chuẩn bị môi trường

Trong terminal mới, thoát Conda nếu đang bật:

```bash
conda deactivate
```

Nếu đang lồng nhiều môi trường, thoát tiếp cho đến khi không còn dùng Python
Conda. ROS Jazzy trên máy dùng Python 3.12; Python Anaconda 3.14 không nạp
được extension `rclpy` đã biên dịch cho 3.12.

Build nếu chưa có `install/setup.bash`, hoặc khi có thay đổi code/đóng gói:

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### Chỉ cài thư viện khi tạo môi trường mới

Laptop hiện đã có venv chạy được ROS và SAC/CUDA; không cần cài lại sau mỗi
lần mở terminal. Nếu cần tạo lại môi trường trên máy khác:

```bash
/usr/bin/python3 -m venv --system-site-packages rl_training/.venv
source rl_training/.venv/bin/activate
python -m pip install --no-cache-dir torch==2.7.1 \
  --index-url https://download.pytorch.org/whl/cu128
python -m pip install --no-cache-dir -r rl_training/requirements.txt
```

Venv kế thừa thư viện ROS hệ thống. Pip có thể báo không gỡ bản hệ thống nằm
ngoài venv; không dùng `sudo pip uninstall` hoặc xóa thư viện Ubuntu để xử lý.
Nếu thiếu `ensurepip`, cài `python3-venv` bằng apt trước khi tạo venv.
Nếu `pip check` báo PyNaCl thiếu cffi:

```bash
python -m pip install --no-cache-dir cffi
```

### Nạp lại trong mỗi terminal dùng để train/đánh giá

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source rl_training/.venv/bin/activate
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_laptop_rl

which python
python --version
ros2 pkg prefix nino_description
```

Python phải ở `rl_training/.venv/bin/python`, phiên bản 3.12; package ROS phải
trỏ vào `install` của workspace này. Terminal kiểm tra topic cũng dùng domain
78. Không source workspace PID cũ vào cùng terminal này.

Kiểm tra import và dependency:

```bash
python - <<'PY'
import sys
sys.path.insert(0, 'rl_training')
from plotting_compat import prepare_matplotlib_namespace
prepare_matplotlib_namespace()
import rclpy, torch, gymnasium, stable_baselines3
from mpl_toolkits.mplot3d import Axes3D
print('Import ROS, RL và vẽ 3D thành công')
PY
python -m pip check
```

`plotting_compat.py` xử lý namespace `mpl_toolkits` giữa Matplotlib Ubuntu và
venv. Các entrypoint train/plot đã gọi nó. Không gỡ Matplotlib hệ thống chỉ
vì một lệnh import trực tiếp báo `Unable to import Axes3D`.

## 2. Kiểm tra CUDA trước khi train

```bash
nvidia-smi
python - <<'PY'
import torch
print('PyTorch:', torch.__version__)
print('CUDA:', torch.version.cuda)
assert torch.cuda.is_available(), 'CUDA chưa sẵn sàng'
print('GPU:', torch.cuda.get_device_name(0))
x = torch.ones((16, 16), device='cuda', requires_grad=True)
(x @ x).sum().backward()
torch.cuda.synchronize()
print('Tính xuôi và lan truyền ngược trên GPU thành công')
PY
```

Train/evaluate mặc định yêu cầu CUDA và kiểm tra tính toán thật. Chọn
`--device cpu` nếu chủ động muốn dùng CPU, hoặc `--device auto` để tự chọn.
Baseline không suy luận mạng nên dùng CPU, nhưng script vẫn cần các thư viện
RL để import. GPU xử lý mạng, nhiều phần Gazebo/ROS vẫn dùng CPU.

## 3. Baseline và smoke

**Dừng các phiên Gazebo/Nav2/PID/teleop khác trước.** Script tự mở mô phỏng và
Nav2, định vị bằng AMCL, gửi đích rồi đóng các tiến trình nó tạo. Không mở
launch thủ công hoặc phát `/cmd_vel` trong lúc chạy các lệnh dưới đây.

### Baseline: không có mô-men bù RL

```bash
python rl_training/train.py --mode baseline \
  --map maps/hall_01.yaml --episodes 3 --seed 1
```

Đích mặc định là `(11, 0)`, ngưỡng lệch đường 0.20 m và giới hạn episode 90 giây
mô phỏng. Thêm `--gui` để xem Gazebo. Chế độ mặc định chỉ chạy server.
Nếu muốn thử ngắn trước, dùng `--episodes 1 --goal-x 3 --max-episode-seconds 60`;
đừng ghép kết quả bài ngắn với evaluate toàn hành lang để làm báo cáo đối chứng.

### Smoke: kiểm tra có cập nhật SAC

```bash
python rl_training/train.py --mode smoke \
  --map maps/hall_01.yaml --steps 32 --device cuda
```

Smoke dùng episode 2 giây, mạng nhỏ và bắt đầu học sau 8 bước. Cần có
`gradient_updates > 0`. Đây là kiểm tra đường dữ liệu, không phải xác nhận
policy đã học vượt gờ. Không dùng model smoke để tiếp tục bài train chuẩn.

## 4. Train mới với số step tùy chọn

```bash
STEPS=10000
CHECKPOINT_EVERY=5000

python rl_training/train.py --mode train \
  --map maps/hall_01.yaml \
  --steps "$STEPS" --checkpoint-every "$CHECKPOINT_EVERY" \
  --seed 1 --device cuda
```

- `--steps`: số bước mục tiêu của model. Step khác episode: một episode gồm
  nhiều step cho đến khi tới đích, thất bại hoặc hết giờ.
- Không có `--resume`: luôn tạo model mới từ đầu và thư mục run mới.
- `--episodes`: dùng cho baseline/evaluate, không điều khiển độ dài chế độ train.
- Config chuẩn bắt đầu cập nhật sau 1.000 bước thu thập; 2.000 bước là bài
  kiểm tra ban đầu, chưa đủ kết luận model tốt.
- Checkpoint định kỳ lưu model cùng replay buffer. Kết thúc bình thường vẫn
  lưu `model.zip`, `replay_buffer.pkl` dù chưa tới mốc checkpoint.

### Gợi ý cho laptop hiện tại

Mốc đã đo: 2.000 bước mất khoảng 5 phút 49 giây theo thời điểm ghi metadata
và result của run trên laptop. Các thời gian dài hơn trong bảng là phép nhân
tuyến tính để tham khảo, chưa phải phép đo hoặc cam kết thời gian hoàn thành.

| Tổng step | Mục tiêu thử | Thời gian tham khảo | Checkpoint gợi ý |
|---:|---|---|---:|
| 2.000 | Kiểm tra train thật | Đã đo khoảng 6 phút | 2.000–5.000 |
| 10.000 | Mốc tiếp theo nên thử | Khoảng 30 phút | 5.000 |
| 20.000 | Đánh giá tiến triển sau 10k | Khoảng 1 giờ | 5.000 |
| 50.000 | Chỉ tăng sau khi mốc trước ổn định | Khoảng 2 giờ 25 phút | 10.000 |
| 100.000 | Lượt dài hơn, cần kiểm tra dung lượng/tải | Khoảng 4 giờ 50 phút | 10.000–20.000 |

Bắt đầu với 1 seed và 3 episode đánh giá mỗi chế độ. Model triển vọng nên
đánh giá 5–10 episode; muốn kiểm tra độ lặp, train 3 model độc lập với seed
1, 2, 3 và dùng baseline/evaluate cùng seed tương ứng. Không lấy 3 episode
của một model thay cho 3 lần train độc lập.

Không tự chạy job 50 triệu bước từ hướng dẫn máy cũ. Số lần reset, thời gian
cập nhật mạng, tải CPU và nhiệt độ làm thời gian thực thay đổi. Nhiều step
hơn không bảo đảm policy tốt hơn.

## 5. Resume: tiếp tục model cũ

Ví dụ model 2.000 bước đã có trên laptop; nếu dùng run khác, đổi đường dẫn:

```bash
PREVIOUS_RUN="$PWD/rl_training/runs/train_20261026_173506_446593"
TARGET_STEPS=10000

python rl_training/train.py --mode train \
  --config "$PREVIOUS_RUN/config.yaml" --map maps/hall_01.yaml \
  --resume "$PREVIOUS_RUN/model.zip" \
  --replay-buffer "$PREVIOUS_RUN/replay_buffer.pkl" \
  --steps "$TARGET_STEPS" --checkpoint-every 5000 \
  --seed 1 --device cuda
```

**`--steps 10000` là tổng số bước đích.** Từ model 2.000 bước sẽ học thêm
8.000 bước. Script in `Target`, `restored`, `remaining` để đối chiếu.
Sau khi xong, model mới nằm trong thư mục `Run:` mới; muốn lên 20.000 thì
resume từ run mới đó, không tiếp tục trỏ về bản 2.000 bước ban đầu.

Giữ cùng config; không ghép model/replay của hai lượt khác nhau. Để dừng,
nhấn `Ctrl+C` một lần và đợi lưu/đóng mô phỏng. Run bị ngắt có thể có cặp
`interrupted_model.zip` và `interrupted_replay.pkl`; dùng đúng cặp đó để resume.

Nếu lấy checkpoint, model và replay phải cùng mốc, ví dụ:

```text
checkpoints/sac_5000_steps.zip
checkpoints/sac_replay_buffer_5000_steps.pkl
```

Dùng hai đường dẫn này cho `--resume` và `--replay-buffer`, vẫn dùng
`--config` của run chứa chúng. Không sửa config cũ để vượt qua kiểm tra resume.

## 6. Đánh giá PI / PID / PI + SAC

Luồng hiện hành dùng đúng ba controller và một checkpoint SAC mỗi lần. Xem [ABLATION_SUITE.md](ABLATION_SUITE.md) cho lệnh chạy và lệnh phân tích riêng. Code/bản hướng dẫn so sánh cũ được lưu trong `../archive/evaluation_legacy_20261030/`; mọi model và kết quả cũ vẫn còn.

## 7. Theo dõi một job train dài

`long_run.py` chỉ quản lý một job resume training; đánh giá thực hiện riêng để tránh trộn tuning, train và evaluation.

```bash
PREVIOUS_RUN="$PWD/rl_training/runs/train_20261026_173506_446593"
JOB_DIR="rl_training/runs/long_laptop_$(date +%Y%m%d_%H%M%S)"
python -u rl_training/long_run.py \
  --resume-train-run "$PREVIOUS_RUN" --map maps/hall_01.yaml \
  --steps 20000 --checkpoint-every 5000 --device cuda \
  --output "$JOB_DIR"
```

`--steps` là tổng bước đích. Checkpoint training vẫn lưu để resume; không tự so nhiều model trong cùng báo cáo.

Theo dõi dashboard ở **terminal 2 mới**, nạp ROS/venv/domain như mục 1 rồi chạy:

```bash
python rl_training/dashboard.py --port 8766
```

Mở `http://localhost:8766` trong trình duyệt, chọn job `long_laptop_...`.
Không cần relay WSL hoặc `--backend-container`. Dashboard hiện tìm các thư
mục `long_*/status.json`: nếu bạn chỉ chạy trực tiếp `train.py`, danh sách job
có thể trống dù lượt train vẫn hoạt động.

Dashboard chỉ đọc log/trạng thái, không phải nút dừng train. Giữ server ở
terminal 2; tắt dashboard không làm job ở terminal 1 dừng. Dừng worker bằng
`Ctrl+C` tại terminal 1 rồi đợi trainer lưu và đóng tiến trình sở hữu.
Không chạy thêm demo điều khiển xe khi job này còn hoạt động.

Job có `status.json`, log từng phase, `evaluations.json` và `evaluations.csv`.
`COMPLETE` là worker đã hoàn tất; đọc `phases.report` để tìm báo cáo. Nếu có
`FAILED`, xem `reason` và log; không tin trạng thái cũ là bằng chứng tiến trình
còn sống sau khi máy bị tắt đột ngột. Tiếp tục bằng model/replay đã lưu của
phase train trong một output mới; không xóa job lỗi.

## 8. Dữ liệu, dung lượng và lỗi hạ tầng

Mỗi lượt ở `rl_training/runs/`, được in sau `Run:`:

- `config.yaml`, `metadata.json`: cấu hình và thông tin nguồn/seed/device.
- `result.json`: kết quả episode hoặc số step/cập nhật mạng.
- `device_verification.json`: actor, critic, forward và optimizer dùng thiết bị nào.
- `episode_.../simulation.log`, `navigation.log`, `steps.jsonl`: dữ liệu/log từng lượt.
- `model.zip`, `replay_buffer.pkl`, `checkpoints/`: model và dữ liệu resume.

Kiểm tra tài nguyên trước các lượt dài:

```bash
df -h /
du -sh rl_training/runs
nvidia-smi
```

Replay buffer của lượt 2.000 bước hiện khoảng 20 MB; nhiều checkpoint sẽ
nhân số bản sao, còn JSON mỗi step tăng theo thời gian. Thư mục runs bị Git
bỏ qua; muốn chuyển máy cần chép riêng model/replay/config và dữ liệu cần giữ.
Không xóa run hay thay cấu hình nguồn/GPU của máy để làm theo hướng dẫn này.

Timeout dữ liệu/clock trong episode là lỗi hạ tầng, không phải reward thất
bại giả. Chương trình lưu `infrastructure_fault.json` và snapshot khi có model,
rồi dừng. Không tự retry episode đang chạy. `exception.json` ghi ngoại lệ khác.
Chỉ khởi động trước goal mới được retry có giới hạn (`startup_attempts=3`,
`startup_progress_timeout=15 s` theo config hiện tại). Khi hết lần thử, xem
log trước khi chạy lại; không nới watchdog để che lỗi.

Reset mỗi episode khởi động lại Gazebo/Nav2 để reset wheel odometry, PI, TF
và AMCL. Không chỉ dịch robot về gốc rồi coi như đã reset đủ trạng thái.
Script không tự tắt các publisher `/clock` hay `/cmd_vel` của phiên ngoài.

Tạo đồ thị một episode, thay đường dẫn mẫu trước khi chạy:

```bash
python rl_training/plot_run.py rl_training/runs/TEN_RUN/episode_TEN_EPISODE
```

### Khôi phục lỗi reset ngày 27/10/2026

Run hướng tới 70.000 step từng dừng ở 55.483 vì Gazebo con còn sống sau khi
wrapper bị launch kết thúc. Các phiên tiếp theo bị trùng clock/controller,
không nhận đủ odometry/IMU. Wrapper đã được sửa để thu hồi đúng nhóm tiến
trình của nó và có bảo vệ khi cha chết. Không tăng timeout hay giới hạn điều khiển.
Bài kiểm tra đóng tiến trình và ba reset ngắn đã qua; điều này chưa bảo đảm
mọi lỗi khởi động ROS/Nav2 sẽ biến mất trong các lượt dài.

Nếu run lưu `infrastructure_fault_model.zip` và `infrastructure_fault_replay.pkl`,
đó là cặp để resume khi dữ liệu lưu đã kiểm tra hợp lệ, không mặc định dùng
`interrupted_model.zip` của Ctrl+C. Giữ config của run đó và tổng đích ban đầu.
Không tự chạy lại nhiều phiên khi còn Gazebo cũ chưa đóng.

## 9. Kiểm chứng hiện có trên laptop và các giới hạn

Các file sau đã có khi rà soát ngày 26/10/2026:

| Run | Kết quả ghi trong run |
|---|---|
| `baseline_20261026_171702_939186` | Đích 3 m, 1 episode SUCCEEDED |
| `baseline_20261026_172427_687335` | Đích 11 m, 3 episode SUCCEEDED |
| `smoke_20261026_173151_928112` | 32 step, 24 cập nhật; actor/critic/forward/optimizer trên RTX 4060, CUDA 12.8 |
| `train_20261026_173506_446593` | 2.000 step, 1.000 cập nhật; có model và replay buffer |

Đây không phải kết quả của các job RTX 5060/50M trong ghi chép máy cũ.
Tại thời điểm rà soát chưa thấy run evaluate hoặc comparison mới trong bản
laptop; cần thực hiện mục 6 để kết luận từ dữ liệu đối chứng.

Các giới hạn còn lại:

- Chưa chứng minh policy tốt hơn PI trên nhiều seed/nhiều địa hình.
- Đánh giá theo AMCL + IMU + encoder; AMCL có thể bỏ sót sai lệch thực. Vận tốc encoder/odom là ước lượng, không chứng minh bánh không trượt.
- Observation chưa chứa đầy đủ lịch sử IMU, tích phân PI hoặc thông tin gờ phía trước.
- Mẫu reward/IMU lấy tại ranh giới step, chưa phải toàn bộ IMU 50 Hz.
- Một môi trường tại một thời điểm; không mở nhiều Gazebo để tăng tốc train hiện tại.

Nếu cần kiểm thử code sau khi sửa, chạy từ root workspace đã nạp môi trường:

```bash
colcon test --packages-select nino_control nino_navigation
colcon test-result --verbose
python -m pytest -q rl_training/tests
```

Những lệnh này là kiểm thử triển khai, không thay thế baseline/evaluate trong Gazebo.
Lịch sử sửa lỗi, kết quả WSL cũ và lệnh demo trên máy trước vẫn có trong
[README_WSL_LICH_SU.md](README_WSL_LICH_SU.md); không dùng số liệu lịch sử đó
làm kết quả huấn luyện của laptop hiện tại.


## Bộ so sánh hiện hành

[ABLATION_SUITE.md](ABLATION_SUITE.md): chạy `run_ablation_suite.py --sac-checkpoint ...`, sau đó phân tích bằng `benchmark_pi_pid_sac.py --pi-dir ... --pid-dir ... --sac-dir ...`. Profile PID không được mặc nhiên coi là tối ưu; xem hồ sơ tuning và tỷ lệ hoàn thành trước khi kết luận.
