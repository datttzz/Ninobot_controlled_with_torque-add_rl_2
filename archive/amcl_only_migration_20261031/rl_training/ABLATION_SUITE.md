# PI / PID / PI + SAC: chạy và đọc kết quả

Hệ thống hiện hành chỉ so sánh **ba controller**, với **một checkpoint SAC** mỗi thí nghiệm. Code cũ nằm trong `archive/evaluation_legacy_20261030/`; checkpoint, replay, log và báo cáo cũ vẫn giữ nguyên. Workspace không có Git nên bản code archive được giữ, không xóa.

## 1. Hai lệnh chính

- `run_ablation_suite.py`: mở Gazebo/Nav2, chạy xe và ghi dữ liệu. Chọn model bằng `--sac-checkpoint`.
- `benchmark_pi_pid_sac.py`: đọc ba thư mục đã chạy và xuất báo cáo offline. Checkpoint/config được xác minh từ metadata của `sac-dir` và byte của archive, không suy số step từ tên thư mục.

PI và PI+SAC luôn giữ Kp=0.30, Ki=0.10, Kd=0. PID dùng `rl_training/config/pid.yaml`. Hồ sơ chọn gain nằm ở `rl_training/config/PID_CALIBRATION.json` nếu đã hiệu chỉnh. **Nếu selected=null thì chưa có ứng viên đạt tiêu chí, không coi profile đang có là PID tối ưu.**

## 2. Chuẩn bị một terminal chạy xe

Dừng các job train, sim, teleop và publisher điều khiển khác trước. Runner kiểm tra job và Gazebo đang hoạt động; chỉ dừng tiến trình do nó tạo.

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source rl_training/.venv/bin/activate
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_fixed3
```

Python phải là venv 3.12. Sau khi cập nhật source, build một lần:

```bash
/usr/bin/colcon build --symlink-install \
  --packages-select nino_control nino_description nino_navigation
source install/setup.bash
```

## 3. Chọn một checkpoint và chạy ba controller

Ví dụ dưới chọn model thực có 50.000 step. Có thể thay bằng checkpoint SB3 SAC `.zip` khác tương thích; mỗi lần vẫn chỉ có một model SAC.

```bash
SAC="$PWD/rl_training/runs/train_20261027_135837_606611/model.zip"

python3 run_ablation_suite.py \
  --sac-checkpoint "$SAC" \
  --episodes 7 --seed 1001 \
  --purpose evaluation --device cpu \
  --output-root rl_training/runs/fixed3_my_evaluation
```

Lệnh trên chạy headless, tuần tự PI 7 → PID 7 → PI+SAC 7. Thêm `--gui` để mở Gazebo GUI. Mỗi lượt tối đa 90 giây mô phỏng; đến đích hoặc thất bại sẽ kết thúc sớm. Thời gian thực còn gồm khởi động, tốc độ mô phỏng và cleanup. `--episodes 10` tương ứng 30 lượt tổng cộng.

Các lựa chọn hữu ích:

- `--dry-run`: kiểm tra file/config/checkpoint và in kế hoạch; chưa nạp policy để inference hoặc mở ROS/Gazebo.
- `--purpose smoke --episodes 1`: kiểm tra nhanh, mặc định tối đa 3 giây mô phỏng. TIME_LIMIT ở đây là chủ ý, không đánh giá chất lượng.
- `--modes PI` hoặc `--modes PID`: chạy riêng để hiệu chỉnh. Báo cáo chính thức chỉ tự tạo khi có đủ cả ba nhóm.
- `--pid-config /path/profile.yaml`: profile PID riêng; không thay PI nền của SAC.
- `--reference-path /path/reference.json`: đường lưu sẵn; mặc định đường map-frame `(0,0)` → `(11,0)` bước 5 cm.
- `--max-episode-seconds 120`: đổi thời gian chung cho cả ba controller, cần output mới.

Root output có `manifest.json`, `reference_path.json`, snapshot Nav2 và các thư mục cố định `PI/`, `PID/`, `PI_SAC/`. Mỗi nhóm có config, controller, metadata, result, log startup và `episode_*/`.

## 4. Phân tích ba thư mục

Sau khi runner ghi đủ số lượt, chạy lệnh phân tích riêng. Có thể dùng terminal chỉ phân tích, không cần ROS/Gazebo:

```bash
python3 benchmark_pi_pid_sac.py \
  --pi-dir rl_training/runs/fixed3_my_evaluation/PI \
  --pid-dir rl_training/runs/fixed3_my_evaluation/PID \
  --sac-dir rl_training/runs/fixed3_my_evaluation/PI_SAC \
  --output-dir benchmark_results/fixed3_my_report
```

Output:

- `report_PI_PID_SAC.html`: độc lập, nhúng dữ liệu, SVG và JavaScript; mở trực tiếp bằng trình duyệt, không cần Internet.
- `summary.csv`: cả năm nhóm chỉ số, Mean/sample SD/Median/P95 và n riêng cho từng metric.
- `comparison_table.tex`: `booktabs`, bảng chính và bảng từng gờ, cùng Mean±SD/Median/P95. Cần `\usepackage{booktabs}`.
- `episodes.csv`, `obstacle_windows.csv`, `obstacle_summary.csv`: dữ liệu chi tiết, cả cửa sổ thiếu/censored.
- `report_data.json`, `audit.json`: schema, coverage, metadata/checkpoint, N/A và lỗi hạ tầng.

Chỉ dùng output-dir mới. Không ghi đè dữ liệu hay báo cáo cũ.

## 5. Đọc kết quả đúng cách

Đọc tỷ lệ hoàn thành trước RMSE. Thành công chính yêu cầu Nav2 SUCCEEDED; cả AMCL và GT có sai số vị trí ≤0.25 m, sai số yaw ≤0.25 rad, và không có mẫu lệch đường quá 0.20 m. Có dòng riêng cho Nav2 báo SUCCEEDED, vì định vị AMCL có thể làm Nav2 báo đến đích trong khi GT còn lệch.

Lượt FAILED/TIME_LIMIT vẫn tính các metric được đo. Thời gian hoàn thành chỉ xét lượt đạt đủ chuẩn. Lượt hạ tầng lỗi/thiếu terminal không bị đổi thành thất bại điều khiển; audit giữ nguyên chúng. RMSE thấp trong một lượt thất bại ngắn không có nghĩa controller tốt hơn.

**Hai loại P95 khác nhau:**

1. `gt_p95_m` / `amcl_p95_m`: P95 |e_y| **trong một episode**, cân thời gian.
2. Cột `p95` trong summary: P95 **giữa các giá trị metric của các episode**. Ví dụ P95 của RMSE qua 7 lượt không phải P95 của từng mẫu e_y.

SD dùng sample SD. Một lượt thì SD=N/A; không đủ dữ liệu được ghi N/A kèm lý do, không điền 0. Đóng góp SAC và vận tốc trung bình chỉ mô tả, không tô đậm “tốt nhất”. Nếu điều kiện/frame/Nav2 khác nhau, báo cáo cảnh báo confound và tắt đánh dấu thứ hạng.

Biểu đồ lỗi/rung/jerk/torque dùng **tiến độ dọc đường s**, không tự coi X=tiến độ. Vạch gờ được chiếu từ tâm dây `cable_bumps` của SDF lên đường tham chiếu. Không phải cảm biến tiếp xúc bánh. GT world và reference map chỉ được nối bằng file `--registration` độc lập đã xác minh, theo convention `p_world = R(theta) * p_map + translation`. Spawn/initialpose cùng tọa độ không chứng minh identity. Metadata mới ghi file, hash, nguồn và bất định; metadata lịch sử thiếu provenance không tự được coi là registration hợp lệ. Thiếu nguồn thì chỉ số map/world là N/A; tuning phụ thuộc GT dừng trước khi chạy.

Đăng ký hiện có cho đúng hall/map/robot: `rl_training/config/registration_hall_01_20261030.json`. Đây là ước lượng độc lập từ scan đứng yên và GT, giữ bất định raster, không phải hiệu chuẩn survey chính xác tuyệt đối. Không fit GT theo AMCL của từng episode. Runner, audit và hai script tuning nhận cùng `--registration`; bước phân tích có thể dùng registration đã nhúng trong metadata mới hoặc chỉ định file đó.

Metric pose cân theo timestamp nguồn riêng. Sai số AMCL–GT đồng bộ là phép tính riêng trên giao timestamp tuyệt đối; nội suy vị trí và unwrap góc, không ngoại suy, max-gap **0.10 s** (5 chu kỳ logger/TF 50 Hz; GT gốc 100 Hz). `simulation_time` chỉ là thời gian episode tương đối. Terminal dùng `terminal_clock_s` cố định sau action kết thúc và các mẫu support thụ động bao thời điểm đó; support không tham gia max/RMSE quỹ đạo hay reward. Log cũ không có clock/bracket terminal đầy đủ được ghi N/A, không thay mẫu cuối bằng mẫu tốt nhất. Báo cáo lịch sử không bị sửa.

Feedback bánh được kiểm tra chung cho PI/PID: timestamp nguồn phải còn mới (tuổi tối đa 0.25 s, cho phép lệch tương lai tối đa 0.10 s). Mẫu trùng không làm mới tuổi; mẫu lùi/không hữu hạn và gap dài reset I/D. Mất feedback chặn PI/PID/residual, đưa mô-men về 0 qua limiter hiện có và ngừng tích phân/phát odometry; khi phục hồi không tích phân qua khoảng mất mẫu. **Direct-torque giữ hành vi override cũ**: vẫn được phép hoạt động qua watchdog/limiter dù thiếu feedback; odometry vẫn đóng băng và diagnostic `direct_torque_without_feedback` báo rõ. Speed guard khi đó chỉ có tốc độ hợp lệ cuối, không được coi là bảo vệ từ phép đo mới. Sửa freshness không loại bỏ trượt bánh hoặc drift vật lý.

Cửa sổ local là ±0.5 m dọc đường; các cửa sổ gần nhau có thể chồng lấn. Settling dùng cửa sổ phục hồi **riêng**, từ khi tâm robot vượt tâm gờ đến tối đa 5 giây, gờ kế hoặc hết episode. Cần |e_y|≤0.05 m, |a_z|≤max(0.5,1.2×P95 trước gờ), tốc độ≥95% median trước gờ, liên tục 0.5 giây. Không quan sát được ổn định thì N/A/censored, không ghi 0.

## 6. Nguồn dữ liệu và hạn chế

`telemetry.jsonl` ghi khoảng 50 Hz, độc lập bước policy. `steps.jsonl` giữ dữ liệu tương tác RL gốc. Ground truth lấy từ OdometryPublisher của Gazebo; AMCL lấy TF map→base_footprint. Thêm plugin đo không đổi mass, ma sát hay mô-men. Gia tốc Z là trục Z thân xe đã trừ vector trọng lực 9.80665 m/s² bằng quaternion. Gyro roll/pitch ở đây là body p/q.

Jerk z: sai phân a_z bằng timestamp IMU, sau đó lọc bậc một nhân quả **5 Hz giống nhau cho mọi nhóm**. Mẫu IMU cùng timestamp không lấy đạo hàm lại; gap >0.1 s reset bộ lọc. Thiếu timestamp IMU thì không dùng timestamp vòng lặp để giả làm dữ liệu đo.

Limiter ratio dùng chênh counter ở vòng điều khiển 1 kHz; thiếu counter mới dùng cờ lấy mẫu và nguồn được giữ trong schema. Mô-men là lệnh cuối sau limiter, không phải cảm biến lực. Đóng góp residual là tỷ lệ lệnh trước limiter cuối; mẫu số ≤1e-6 N·m bị loại thay vì chia số rất nhỏ. Cảm biến ghi mẫu mới nhất, không hứa đồng bộ phần cứng tuyệt đối.

Nguồn plugin: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc

## 7. Hiệu chỉnh PID có giới hạn

```bash
python3 rl_training/tune_pid.py \
  --output-root rl_training/runs/pid_tuning_my_run \
  --budget-minutes 60
```

Protocol được ghi **trước khi chạy**: 6 ứng viên, 1 lượt sàng lọc 30 giây/ứng viên; giữ tối đa 2 finalist, so với PI bằng 2 lượt đầy đủ mỗi profile ở tập seed khác. Dữ liệu tuning giữ riêng, không trộn vào evaluation. Seed chỉ gắn nhãn Gym, không bảo đảm cùng nhiễu Gazebo/AMCL.

Tiêu chí chọn: giảm RMSE vận tốc bánh ít nhất 5%; tỷ lệ hoàn thành/Nav2 không thấp hơn PI; GT/AMCL RMSE không tăng quá 5 mm; slew không tăng quá 1 điểm phần trăm; saturation không tăng quá 0.1 điểm phần trăm; không vượt torque 2 N·m. Đây là biên dung sai đặt trước, không phải kiểm định thống kê hay chứng minh tối ưu toàn cục. Không ứng viên nào đạt thì **giữ nguyên profile**, ghi rõ selected=null.

D trên vận tốc đo: `D_raw=-Kd*(omega_k-omega_prev)/dt`, `alpha=tau_D/(tau_D+dt)`. Profile lưu tau_D theo giây, alpha thay đổi đúng dt. Reset D/I khi timestamp/vận tốc không hợp lệ, clock lùi, mất mẫu lâu, dừng hoặc reset episode. Kd=0 đi qua phép tính PI cũ.

Giữ giới hạn gốc: velocity torque ±2 N·m, hard torque ±12 N·m, slew 10 N·m/s, wheel speed 12 rad/s, acceleration 12 rad/s²; watchdog và collision monitor giữ nguyên. Không yêu cầu slew ratio=0 bằng cách nới limiter.

Budget tối đa 60 phút, dành thời gian cleanup; `protocol.json`, `state.json` và resource journal lưu tiến độ. Dừng khi RAM khả dụng <2 GiB, disk <3 GiB, hoặc nhiệt độ cảm biến ≥95°C qua ba lần kiểm tra liên tiếp. Chỉ một Gazebo headless. Mọi log và thử nghiệm bị ngắt đều giữ lại.

## 8. Dừng và chạy tiếp

Ctrl+C một lần, đợi cleanup rồi mới tắt máy. Chạy lại nguyên lệnh evaluation với `--resume`, cùng output/config/model/code/domain/partition. Lượt hoàn tất, kể cả FAILED/TIME_LIMIT, được giữ nguyên; chỉ làm lại lượt chưa hoàn tất. Nếu đổi code hay điều kiện, tạo output mới.

Tuning hỗ trợ `--resume` trong cùng boot và ngân sách wall-clock ban đầu, không khởi động lại đồng hồ 60 phút. Sau reboot hoặc hết budget, giữ nguyên log và tạo protocol mới nếu muốn thử tiếp.
