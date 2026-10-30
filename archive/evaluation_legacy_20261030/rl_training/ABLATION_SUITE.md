# Chạy tự động PI, PID và nhiều checkpoint SAC

`run_ablation_suite.py` ở **root workspace** chạy các nhóm tuần tự bằng Gazebo + Nav2 thật, sau đó gọi bộ benchmark offline để xuất CSV, LaTeX và HTML. Không train lại, không sửa reward, checkpoint hay kết quả cũ.

## 1. Điều cần biết trước khi chạy

Repo ban đầu chỉ có PI: `Kp=0.30`, `Ki=0.10`. Chưa có bộ PID đã hiệu chỉnh. Nhánh PID mới hỗ trợ `Kd>0`, nhưng **không có gain PID tối ưu được cài sẵn**. Chọn `--modes PI PID SAC` cần thêm `--pid-config` chứa gain bạn đã hiệu chỉnh bằng các lượt riêng. Có thể chạy ngay PI và SAC khi chưa có gain PID.

Checkpoint của repo là **Stable-Baselines3 SAC `.zip`**, không phải trọng số PyTorch `.pt/.pth`. Runner từ chối `.pt/.pth` vì không biết kiến trúc policy, observation normalization và action scale của chúng. Không đổi đuôi file. Nếu bạn sở hữu đối tượng SB3 SAC đã nạp đúng, dùng `model.save(...)` để xuất archive SB3 tương thích. Model ngoài repo cần adapter và metadata phù hợp trước khi đánh giá.

Mỗi checkpoint cần `config.yaml` và `metadata.json` nguồn trong cùng thư mục run (hoặc thư mục cha của `checkpoints/`). Runner đọc `num_timesteps` trong archive, SHA256 và metadata, sau đó thực sự nạp SAC để kiểm tra spaces trước khi mở môi trường của nhóm đó. Không suy số step từ tên file. Metadata cũ không ghi gain bánh lúc train: runner giữ PI 0.30/0.10 và công khai giới hạn bằng chứng này trong manifest.

## 2. Chuẩn bị terminal

Đóng Gazebo và các phiên điều khiển trước đó. Dùng **một terminal duy nhất để chạy suite**; không tự mở thêm sim, teleop hoặc publisher `/cmd_vel`.

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
# Nếu còn Conda, chạy conda deactivate cho tới khi ra khỏi môi trường đó.
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source rl_training/.venv/bin/activate

export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_laptop_rl

which python3
python3 --version
```

Python phải ở `rl_training/.venv/bin/python3`, phiên bản 3.12; không dùng Python 3.14 của Anaconda.

Sau lần cập nhật code này, build một lần bằng Python hệ thống:

```bash
/usr/bin/colcon build --symlink-install \
  --packages-select nino_control nino_description nino_navigation
source install/setup.bash
```

## 3. Kiểm tra nhanh PI và SAC trước

Hai checkpoint có thật đã xác minh: run `train_20261027_135837_606611` có 50.000 step; run `train_20261027_173345_545739` có 70.000 step. Đặt biến đường dẫn để dễ thay:

```bash
CKPT50="$PWD/rl_training/runs/train_20261027_135837_606611/model.zip"
CKPT70="$PWD/rl_training/runs/train_20261027_173345_545739/model.zip"
```

Chỉ đọc cấu hình/model và in kế hoạch, chưa mở Gazebo:

```bash
python3 run_ablation_suite.py \
  --modes PI SAC \
  --checkpoints "$CKPT50" "$CKPT70" \
  --episodes 7 \
  --dry-run
```

Chạy **kiểm tra kỹ thuật ngắn**, mỗi nhóm một episode tối đa 3 giây mô phỏng, có GUI:

```bash
python3 run_ablation_suite.py \
  --modes PI SAC \
  --checkpoints "$CKPT50" \
  --episodes 1 --purpose smoke --gui \
  --output-root ./ablation_runs_smoke
```

Kết quả `TIME_LIMIT` ở smoke là bình thường: thời gian quá ngắn để đi hết 11 m. Không dùng báo cáo smoke để kết luận bộ điều khiển tốt hơn. Thư mục output phải mới; giữ riêng dữ liệu smoke, hiệu chỉnh và đánh giá chính thức.

## 4. Chạy chính thức PI và nhiều checkpoint

```bash
python3 run_ablation_suite.py \
  --modes PI SAC \
  --checkpoints "$CKPT50" "$CKPT70" \
  --episodes 7 --seed 1 --gui \
  --device cpu \
  --output-root ./ablation_runs_pi_sac_7
```

Đây là **21 episode**: PI 7, PI+SAC 50k 7, PI+SAC 70k 7. Thêm đường dẫn sau `--checkpoints` để thêm model. Dùng `--episodes 10` nếu muốn 10 lượt mỗi nhóm. Với laptop, nên kiểm tra 1 lượt/ngóm trước, sau đó dùng 7 lượt cho đánh giá ban đầu; 10 lượt giúp quan sát độ phân tán tốt hơn nhưng tốn thêm thời gian. Thời gian thực còn gồm khởi động và dọn Gazebo/Nav2 ở mỗi lượt.

CPU phù hợp inference mạng nhỏ; không thực hiện cập nhật gradient. Có thể dùng `--device cuda` với môi trường CUDA đã kiểm tra. Bỏ `--gui` để giảm tải render. GUI chỉ mở Gazebo; RViz không tự mở.

## 5. Thêm PID một cách có căn cứ

Tạo file gain riêng:

```bash
cp rl_training/config/pid_gains.template.yaml rl_training/config/my_tuned_pid.yaml
nano rl_training/config/my_tuned_pid.yaml
```

File có đúng bốn khóa: `velocity_kp`, `velocity_ki`, `velocity_kd`, `derivative_filter_seconds`. Điền số hữu hạn, gain không âm, **Kd và thời hằng lọc phải >0**. Mẫu cố ý để hai giá trị là `null`, nên runner sẽ yêu cầu bạn điền. Không có nghĩa gain trong một kiểm thử phần mềm là gain đã hiệu chỉnh.

Nên thử PID bằng các output riêng, giữ nguyên giới hạn và Nav2. Ghi từng bộ gain, thời hằng lọc, sai số bám vận tốc bánh, dao động/đỉnh vận tốc, tỷ lệ bão hòa và slew limit; chọn gain trước khi xem kết quả chính thức. Không chọn chỉ theo reward hoặc một lượt đến đích. Giữ lại mọi lượt hiệu chỉnh nhưng **không trộn vào 7 lượt đánh giá chính**. Chỉ đạt 7/7 cũng chưa chứng minh PID tốt hơn.

Chạy riêng PID để kiểm tra sau khi điền gain:

```bash
python3 run_ablation_suite.py \
  --modes PID \
  --pid-config rl_training/config/my_tuned_pid.yaml \
  --episodes 1 --gui \
  --output-root ./ablation_pid_calibration_trial01
```

Sau khi khóa gain, chạy đủ ba nhóm:

```bash
python3 run_ablation_suite.py \
  --modes PI PID SAC \
  --checkpoints "$CKPT50" "$CKPT70" \
  --pid-config rl_training/config/my_tuned_pid.yaml \
  --episodes 7 --seed 1 --gui \
  --device cpu \
  --output-root ./ablation_runs_all_7
```

Tổng cộng **28 episode**. Các gain PID chỉ áp dụng PID; PI và mọi PI+SAC luôn dùng Kd=0 và PI nền nguyên bản. Runner không sửa YAML controller gốc. Nhánh D sử dụng vận tốc đo, không dùng đạo hàm setpoint:

```text
alpha = dt_đo / (thời_hằng_lọc + dt_đo)
d_lọc = d_lọc_trước + alpha * ((v_đo - v_đo_trước)/dt_đo - d_lọc_trước)
torque = Kp*error + Ki*integral - Kd*d_lọc
```

Mẫu có cùng timestamp không bị đạo hàm lại; clock lùi hoặc mất mẫu dài làm reset bộ lọc. Episode mới khởi động controller mới nên tích phân và D được reset. Giữ integral clamp, giới hạn mô-men, slew, tốc độ bánh, watchdog và collision monitor hiện có. Không thay chính sách anti-windup của PI đã dùng khi train.

## 6. Điều kiện công bằng và quỹ đạo chung

Tất cả nhóm dùng cùng map, world `long_hall`, spawn vật lý `(0,0,0)` và hướng 0, initial pose AMCL `(0,0,0)`, goal, Nav2 YAML, bước RL, giới hạn thời gian và chấp hành. Mỗi episode khởi động lại toàn bộ simulation/Nav2 do runner sở hữu.

Mặc định runner tạo một đường tâm **trong frame `map`** từ `(0,0)` tới goal `(11,0)`, cách điểm khoảng 5 cm, ghi thành `reference_path.json`. Đây là thay đổi giao thức đánh giá so với các log cũ dùng đường planner riêng từng lượt. Không so trực tiếp số mới và báo cáo cũ như thể chỉ khác controller. Đường tâm mặc định dành cho `hall_01`; không phải chứng nhận đường không va chạm cho bản đồ tùy ý. Nav2 FollowPath, velocity smoother và collision monitor vẫn hoạt động. Đường không được replan để cứu một nhóm.

Dùng đường riêng đã kiểm tra bằng `--reference-path /đường/dẫn/reference.json`, schema:

```json
{"frame_id":"map","poses":[{"x":0.0,"y":0.0,"yaw":0.0},{"x":1.0,"y":0.0,"yaw":0.0},{"x":11.0,"y":0.0,"yaw":0.0}]}
```

Nên cung cấp các điểm đủ dày và đúng bản đồ. Điểm đầu phải ở spawn, điểm cuối trùng goal trong `--config`, yaw cuối 0 theo giao diện môi trường hiện tại. Runner so SHA256 hình học tham chiếu ở mỗi episode. `planning_requests=0` là chủ ý khi dùng đường lưu sẵn; không phải planner bị hỏng.

`--seed 1 --episodes 7` gắn nhãn Gym seed 1..7. **Không kiểm soát cùng nhiễu vật lý Gazebo hoặc AMCL**, nên không tuyên bố cùng seed là cùng nhiễu. Thứ tự nhóm chạy đúng thứ tự `--modes`, sau đó thứ tự checkpoint; không ngẫu nhiên hóa thứ tự thí nghiệm.

## 7. Theo dõi, dừng và chạy tiếp

Runner in tên nhóm và đường dẫn `runner.log`. Muốn xem trực tiếp, mở terminal thứ hai **chỉ đọc log**, ví dụ:

```bash
tail -f ablation_runs_all_7/PI_only/runner.log
```

Các thư mục SAC có tên `PI_SAC_<step>steps_<8 ký tự hash>`, xem bằng:

```bash
ls ablation_runs_all_7
cat ablation_runs_all_7/status.json
```

Bấm `Ctrl+C` **một lần**, chờ runner dọn các phiên do nó tạo rồi mới tắt máy. Chạy lại **nguyên lệnh trước** và thêm `--resume`. Ví dụ nhóm PI/SAC:

```bash
python3 run_ablation_suite.py \
  --modes PI SAC \
  --checkpoints "$CKPT50" "$CKPT70" \
  --episodes 7 --seed 1 --gui --device cpu \
  --output-root ./ablation_runs_pi_sac_7 \
  --resume
```

Cần khai báo lại biến `CKPT50/CKPT70` khi mở terminal mới. Không đổi gain, config, code, môi trường ROS/GZ hoặc checkpoint giữa hai lần; runner kiểm tra manifest. Nếu cần đổi điều kiện, tạo output-root mới. Episode hoàn tất (kể cả FAILED/TIME_LIMIT) được giữ và tính vào tổng; chỉ episode chưa hoàn tất mới được làm lại. Log bị ngắt và mọi lần retry vẫn giữ để audit, không tính thành thất bại điều khiển.

Khi lỗi hạ tầng sau retry, suite dừng để bạn xử lý, không tiếp tục tạo dữ liệu lỗi cho các nhóm sau, không tự kill Gazebo bên ngoài. Giữ cả `startup_recovery.jsonl`, `attempt_events.jsonl`, exception và session log. Sau khi xử lý nguyên nhân mà không thay đổi giao thức, dùng `--resume`.

## 8. Kết quả và cách đọc

Sau khi đủ số lượt mọi nhóm:

```text
output-root/
  manifest.json                 # Điều kiện chung, SHA nguồn/model, các giới hạn bằng chứng
  reference_path.json           # Cùng hình học cho mọi lượt
  nav2.yaml                     # Snapshot Nav2
  status.json
  PI_only/                      # config/controller/metadata/result và log episode
  PID_only/                     # Có khi chọn PID
  PI_SAC_50000steps_<hash>/
  PI_SAC_70000steps_<hash>/
  report/
    comparison_summary.csv
    comparison_table.tex
    benchmark_report.html
    episode_metrics.csv
    audit.json
```

Mở `report/benchmark_report.html` bằng trình duyệt, không cần Internet. Chọn nhóm/episode để xem biểu đồ. CSV có mean/std/n từng metric; độ lệch chuẩn là sample SD. Giữ cả lượt FAILED/TIME_LIMIT; thời gian hoàn thành chỉ xét SUCCEEDED. Xem cả tỷ lệ đến đích, lỗi ngang, tốc độ bánh, RMS gia tốc, torque và giới hạn; không chỉ nhìn reward.

Lỗi ngang dùng pose **AMCL map→base_footprint** so với đường map chung, không phải ground truth. Log mới ghi thêm pose AMCL, gyro, wheel target/velocity và cờ limiter. Ground truth chỉ ghi nếu topic có dữ liệu mới; world/bridge hiện tại không bảo đảm có `/ground_truth/odom`, nên N/A là đúng. Không âm thầm dùng encoder odom làm ground truth. Tín hiệu khác nhau là mẫu mới nhất tại bước RL, không phải tất cả cảm biến đồng bộ chính xác. Cờ saturation là chạm ngưỡng ở tầng điều khiển vận tốc; cờ slew là yêu cầu vượt bước mô-men cho phép. Tỷ lệ trong báo cáo là tỷ lệ theo mẫu RL cân thời gian, không phải đếm toàn bộ chu kỳ 1 kHz.

Có thể gọi lại `evaluate_and_compare.py` với các thư mục nhóm để thêm cửa sổ gờ hoặc gộp với run tương thích. Dùng pose AMCL X hay progress phải ghi rõ hệ tọa độ; không đồng nhất map X với world X nếu chưa xác minh transform. Xem [BENCHMARK_SUITE.md](BENCHMARK_SUITE.md).

Wrapper Gazebo cũng thu hồi server do GUI fork rồi đổi session (`setsid`). Runner kiểm tra tiến trình trước và sau mỗi nhóm; nếu server còn sống, suite dừng và ghi lỗi hạ tầng. Đây là kiểm tra thêm bên cạnh ROS graph, vì server có thể đổi tên tiến trình thành `gz sim server` hoặc ở partition khác. Không dùng `pkill gz` để dọn phiên ngoài.
