# PI / PID / PI + SAC: chạy và đọc kết quả

Hệ thống hiện hành chỉ so sánh **ba controller**, với **một checkpoint SAC** mỗi thí nghiệm. Code cũ nằm trong `archive/evaluation_legacy_20261030/`; checkpoint, replay, log và báo cáo cũ vẫn giữ nguyên. Các bản archive được giữ để tra cứu và khôi phục code cũ.

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
- `--modes PI` hoặc `--modes PID`: chạy riêng và ghi log. Runner không tự xuất báo cáo; bước phân tích là lệnh riêng khi cần.
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

Đọc tỷ lệ hoàn thành trước RMSE. Thành công chính yêu cầu Nav2 SUCCEEDED; AMCL có sai số vị trí ≤0.25 m, sai số yaw ≤0.25 rad, và không có mẫu lệch đường quá 0.20 m. Giữ riêng outcome Nav2 và AMCL task pass. Đây là đánh giá theo định vị và cảm biến, không phải phép đo vị trí thực độc lập.

Lượt FAILED/TIME_LIMIT vẫn tính các metric được đo. Thời gian hoàn thành chỉ xét lượt đạt đủ chuẩn. Lượt hạ tầng lỗi/thiếu terminal không bị đổi thành thất bại điều khiển; audit giữ nguyên chúng. RMSE thấp trong một lượt thất bại ngắn không có nghĩa controller tốt hơn.

**Hai loại P95 khác nhau:**

1. `amcl_p95_m`: P95 |e_y| **trong một episode**, cân thời gian.
2. Cột `p95` trong summary: P95 **giữa các giá trị metric của các episode**. Ví dụ P95 của RMSE qua 7 lượt không phải P95 của từng mẫu e_y.

SD dùng sample SD. Một lượt thì SD=N/A; không đủ dữ liệu được ghi N/A kèm lý do, không điền 0. Đóng góp SAC và vận tốc trung bình chỉ mô tả, không tô đậm “tốt nhất”. Nếu điều kiện/frame/Nav2 khác nhau, báo cáo cảnh báo confound và tắt đánh dấu thứ hạng.

Biểu đồ và cửa sổ dùng tiến độ dọc đường s tính từ AMCL trong map. Tâm gờ phải được chỉ định trong map bằng `--obstacle-x` (map Y=0 trên đường hành lang hiện tại). Không suy tọa độ map từ world. Thiếu vị trí gờ trong map thì phân tích cửa sổ N/A. Cửa sổ là vùng phân tích, không phải cảm biến tiếp xúc.

Pose lấy TF `map→base_footprint` do AMCL kết hợp odometry encoder cung cấp. Metric pose cân theo timestamp TF riêng, IMU/controller/odom theo timestamp nguồn tương ứng. Terminal nội suy SE(2) tại `terminal_clock_s` cố định, unwrap góc, không ngoại suy hoặc qua gap lớn hơn **0,10 s** (5 chu kỳ TF/logger 50 Hz). Log thiếu clock/bracket terminal được ghi N/A. Mẫu support thụ động không thay outcome/reward và không tham gia RMSE/max quỹ đạo. `simulation_time` là thời gian episode tương đối, khác ROS clock tuyệt đối.

Feedback bánh được kiểm tra chung cho PI/PID: timestamp nguồn phải còn mới (tuổi tối đa 0.25 s, cho phép lệch tương lai tối đa 0.10 s). Mẫu trùng không làm mới tuổi; mẫu lùi/không hữu hạn và gap dài reset I/D. Mất feedback chặn PI/PID/residual, đưa mô-men về 0 qua limiter hiện có và ngừng tích phân/phát odometry; khi phục hồi không tích phân qua khoảng mất mẫu. **Direct-torque giữ hành vi override cũ**: vẫn được phép hoạt động qua watchdog/limiter dù thiếu feedback; odometry vẫn đóng băng và diagnostic `direct_torque_without_feedback` báo rõ. Speed guard khi đó chỉ có tốc độ hợp lệ cuối, không được coi là bảo vệ từ phép đo mới. Sửa freshness không loại bỏ trượt bánh hoặc drift vật lý.

Cửa sổ local là ±0.5 m dọc đường; các cửa sổ gần nhau có thể chồng lấn. Settling dùng cửa sổ phục hồi **riêng**, từ khi tâm robot vượt tâm gờ đến tối đa 5 giây, gờ kế hoặc hết episode. Cần |e_y|≤0.05 m, |a_z|≤max(0.5,1.2×P95 trước gờ), tốc độ≥95% median trước gờ, liên tục 0.5 giây. Không quan sát được ổn định thì N/A/censored, không ghi 0.

## 6. Nguồn dữ liệu và hạn chế

`telemetry.jsonl` ghi khoảng 50 Hz, độc lập bước policy. `steps.jsonl` giữ dữ liệu tương tác RL gốc. Pose lấy từ TF map→base_footprint; vận tốc thân lấy `/odom` là ước lượng từ encoder, không chứng minh xe không trượt. Mô-men là lệnh controller sau limiter, không phải cảm biến lực. Gia tốc Z là trục Z thân xe đã trừ vector trọng lực 9.80665 m/s² bằng quaternion. Gyro roll/pitch ở đây là body p/q.

Jerk z: sai phân a_z bằng timestamp IMU, sau đó lọc bậc một nhân quả **5 Hz giống nhau cho mọi nhóm**. Mẫu IMU cùng timestamp không lấy đạo hàm lại; gap >0.1 s reset bộ lọc. Thiếu timestamp IMU thì không dùng timestamp vòng lặp để giả làm dữ liệu đo.

Limiter ratio dùng chênh counter ở vòng điều khiển 1 kHz; thiếu counter mới dùng cờ lấy mẫu và nguồn được giữ trong schema. Mô-men là lệnh cuối sau limiter, không phải cảm biến lực. Đóng góp residual là tỷ lệ lệnh trước limiter cuối; mẫu số ≤1e-6 N·m bị loại thay vì chia số rất nhỏ. Cảm biến ghi mẫu mới nhất, không hứa đồng bộ phần cứng tuyệt đối.


## 7. PID: tái sử dụng log và chỉ hai lượt xác minh mới

```bash
python3 rl_training/calibrate_pid_measured.py \
  --output-root rl_training/runs/pid_amcl_verify_new \
  --budget-minutes 30
```

`tune_pid.py` gọi cùng entrypoint này, không còn tự chạy sáu ứng viên và PI screening. Tập gain được khóa trong protocol trước khi đọc metric. Luồng đọc lại log lịch sử tương thích map, đường, Nav2, vận tốc và giới hạn; chỉ dùng nguồn AMCL/IMU/encoder, không dùng kết luận cũ để loại gain. Thiếu bracket terminal vẫn là chưa đủ bằng chứng, không suy pose tốt nhất.

Không mở mô phỏng mới cho sàng lọc. Khóa một gain, sau đó chỉ chạy **hai episode PID-only** ở seed 9301–9302. Không dùng hai lượt này để đổi gain. Chỉ lưu profile được chọn nếu cả hai hoàn thành theo AMCL (xy≤0.25 m, |yaw|≤0.25 rad, hành lang≤0.20 m), ổn định và không lỗi dữ liệu/feedback; nếu không thì giữ profile cũ. Đây là phạm vi xác minh nhỏ theo yêu cầu, không chứng minh tối ưu hoặc độ tin cậy dài hạn.

Xếp từ điển: RMSE bám vận tốc bánh → RMSE ngang AMCL → RMS gia tốc ngang IMU body ay đã trừ trọng lực. Theo dõi slew/saturation, không yêu cầu slew bằng 0 và không ép hơn PI ở mọi chỉ số. Roll/pitch peak≤0.5 rad, tốc độ bánh đo peak≤24 rad/s, target≤12 rad/s, lệnh torque peak≤2 N·m; coverage nguồn≥99% và feedback hợp lệ. Giữ nguyên mọi giới hạn vận hành và tốc độ tham chiếu.

D trên vận tốc đo: `D_raw=-Kd*Δomega/dt`, `alpha=tau_D/(tau_D+dt)`. Không đổi code D, anti-windup, watchdog hay freshness. Budget tối đa 30 phút, một Gazebo headless, giữ toàn bộ log và cấu hình; dừng theo các giới hạn tài nguyên sẵn có.

## 8. Dừng và chạy tiếp

Ctrl+C một lần, đợi cleanup rồi mới tắt máy. Chạy lại nguyên lệnh evaluation với `--resume`, cùng output/config/model/code/domain/partition. Lượt hoàn tất, kể cả FAILED/TIME_LIMIT, được giữ nguyên; chỉ làm lại lượt chưa hoàn tất. Nếu đổi code hay điều kiện, tạo output mới.

Luồng hai lượt xác minh không tự chạy thêm khi thất bại hoặc hết ngân sách. Giữ log hiện tại; một đợt thử mới cần protocol/output mới.

Code đo/đăng ký cũ được giữ trong `archive/amcl_only_migration_20261031/` để tra lịch sử, không thuộc đường import hiện hành. Log, checkpoint và báo cáo đã ghi không bị sửa. Tiêu chí hiệu chỉnh cũ không tự được công nhận theo tiêu chí AMCL-only mới; tool chặn dùng lại log/schema cũ cho tuning.
