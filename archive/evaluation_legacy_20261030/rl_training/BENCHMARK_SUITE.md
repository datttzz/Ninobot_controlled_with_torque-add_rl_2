# Multi-Model & Multi-Checkpoint Benchmark Suite

Công cụ **offline đọc log**, không chạy Gazebo/ROS, không nạp torch, không train hoặc sửa controller. Mỗi nhãn là một nhóm thí nghiệm do bạn cung cấp. Có thể đặt PI, PID, PI+SAC, ablation reward hoặc tên bất kỳ; công cụ không giới hạn hai nhóm và không suy ra controller từ tên file.

Khảo sát định dạng và vị trí dây thực của repo: [BENCHMARK_LOG_SURVEY.md](BENCHMARK_LOG_SURVEY.md).

## Chạy từ root workspace

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source rl_training/.venv/bin/activate

python3 evaluate_and_compare.py \
  --runs "PI:/path/to/pi_logs" \
         "PID:/path/to/pid_logs" \
         "PI+SAC (50k):/path/to/sac50k" \
         "PI+SAC (70k):/path/to/sac70k" \
  --obstacle-x 3.0 6.0 9.0 \
  --window-radius 0.5 \
  --max-deviation 0.20 \
  --output-dir ./benchmark_results/my_comparison
```

Thay các path ví dụ bằng thư mục thật. Nhãn có dấu cách cần đặt trong dấu nháy. Có thể lặp lại cùng nhãn với các thư mục khác nhau để gộp nhiều run vào một nhóm. Không truyền cùng log hai lần hoặc thư mục cha và con trùng nhau. Công cụ từ chối ghi đè output đã tồn tại.

Không cần source ROS hoặc mở Gazebo. Dependencies dùng các gói có sẵn trong repo: NumPy và PyYAML. HTML không cần Plotly/CDN/Internet.

Ví dụ chạy ngay với ba bộ dữ liệu 7 episode đang có:

```bash
python3 evaluate_and_compare.py \
  --runs "PI:rl_training/runs/baseline_20261027_232307_060134" \
         "PI+SAC (50k):rl_training/runs/evaluate_20261027_233155_548556" \
         "PI+SAC (70k):rl_training/runs/evaluate_20261027_234228_634376" \
  --obstacle-x 3.0 6.0 9.0 \
  --obstacle-axis progress \
  --window-radius 0.5 \
  --max-deviation 0.20 \
  --output-dir ./benchmark_results/three_models_example
```

Ví dụ trên chủ động chọn `progress`: các tâm 3/6/9 khi đó là khoảng cách dọc đường, **không phải tọa độ X hay sự kiện tiếp xúc vật lý**. Log native cũ chưa có X/Y. Nếu dùng mặc định `--obstacle-axis x`, metric quanh gờ sẽ N/A ở các log này; metric toàn episode vẫn tính bình thường. `--obstacle-axis ground-truth-x` dùng cột GT X nếu có.

Tâm cửa sổ phải cùng hệ tọa độ với cột pose được chọn. Tọa độ tâm dây trong SDF là world frame; không tự suy ra transform world→AMCL map. Dây nghiêng nên vị trí bánh tiếp xúc còn phụ thuộc Y, kích thước robot và góc hướng. `3,6,9` cũng không phải các tâm dây đúng của world long_hall hiện tại; xem bảng khảo sát.

## Định dạng đầu vào

Hỗ trợ thư mục run native có `episode_*/steps.jsonl`, thư mục một episode có `steps.jsonl`, hoặc CSV telemetry. Nếu thư mục chứa nhiều run con có metadata/episode, công cụ tìm các run con đó. Trong một episode ưu tiên `steps.jsonl`, sau đó `steps.csv`, `telemetry.csv`, `trajectory.csv`. Các CSV riêng cho nhiều sensor cần được đồng bộ/gộp cột trước; không tự ghép theo timestamp gần nhất.

Thư mục CSV phẳng có thể chứa mỗi file là một episode. Một CSV chứa nhiều episode cần cột `episode_id` (hoặc `episode_index`, `episode`). Mỗi episode phải có ít nhất hai timestamp và thời gian lấy mẫu dương. File `monitor.csv` có `r,l,t` chỉ là tổng hợp episode, không đủ để suy ra các metric cảm biến nên được ghi rõ trong audit.

Tên cột CSV chuẩn khuyến nghị (mỗi mẫu một dòng):

| Nội dung | Cột / định dạng |
|---|---|
| Timestamp giây | `simulation_time` (từ đầu episode), hoặc `time_s`/`timestamp`/`timestamp_s` |
| Timestamp nanosecond | `timestamp_ns` |
| Khoảng thời gian mẫu | `step_duration` hoặc `dt_s`/`dt`, đơn vị giây |
| Kết quả cuối episode | `outcome`: SUCCEEDED / FAILED / TIME_LIMIT, chỉ ở dòng cuối; `reason` tùy chọn |
| Lỗi ngang | `cross_track` hoặc `cross_track_error_m` / `cross_track_error` / `e_y` (m) |
| Lỗi hướng | `heading_error` / `heading_error_rad` / `e_heading` (rad) |
| Tiến triển dọc đường | `progress` / `progress_m` / `path_progress_m` (m) |
| Pose AMCL | `amcl_x_m`, `amcl_y_m`, `amcl_yaw_rad` |
| Pose ground truth | `ground_truth_x_m`, `ground_truth_y_m` |
| Gia tốc đã xử lý trọng lực | `ax,ay,az` hoặc `a_x,a_y,a_z` hoặc `acceleration_x,acceleration_y,acceleration_z` (m/s²) |
| Angular velocity | `gyro_x,gyro_y,gyro_z` hoặc `angular_velocity_x,angular_velocity_y,angular_velocity_z` (rad/s) |
| Góc thân xe | `roll`/`roll_rad`, `pitch`/`pitch_rad` (rad), không phải angular velocity |
| Vận tốc/target bánh | `wheel_left_radps,wheel_right_radps,target_left_radps,target_right_radps` |
| PI torque | `pi_torque_left_nm,pi_torque_right_nm` |
| Residual đã chấp nhận | `residual_left_nm,residual_right_nm` |
| Torque cuối | `total_torque_left_nm,total_torque_right_nm` |
| Cờ limiter được ghi thật | `saturation_flags`, `slew_rate_flags`: bool/0/1 hoặc JSON array bool cho các bánh |
| Reward mỗi step | `reward` hoặc `step_reward` |
| Thành phần reward | `reward_terms` JSON object hoặc cột `reward_progress`, `reward_vibration`, ... |

Các vector native (`acceleration`, `base_torque`, `residual_applied`, `total_torque`, `wheel_velocity`, `wheel_target`, `gyro`) cũng được chấp nhận dưới dạng JSON array trong JSONL hoặc ô CSV có quote đúng. Gia tốc nhập phải đã trừ trọng lực hoặc phải ghi rõ đó là dữ liệu chưa hiệu chỉnh khi diễn giải kết quả; công cụ không tự gỡ trọng lực nếu thiếu orientation. GT và AMCL cần cùng hệ quy chiếu và cùng thời điểm lấy mẫu để sai số định vị có ý nghĩa.

Để trống cột chưa đo, không điền 0 thay cho dữ liệu thiếu. Nếu không có outcome cuối, episode được đánh dấu INCOMPLETE và không đưa vào mẫu số success hoặc thống kê chính. Không suy ra thành công từ việc lỗi ngang nhỏ. Không dùng các CSV summary cũ để giả tạo chuỗi đo.

## Chỉ số và cách thống kê

Mỗi chỉ số nhóm có `mean`, `std` mẫu và `n` episode có dữ liệu. Không gộp tất cả sample của mọi episode rồi để episode dài chiếm trọng số lớn hơn. FAILED và TIME_LIMIT vẫn nằm trong các trung bình chính. Thời gian hoàn thành chỉ tính trên SUCCEEDED. INCOMPLETE/log hỏng và phiên không có mẫu được liệt kê riêng.

- Success rate, số thành công/thất bại/hết giờ, khoảng Wilson 95% cho tỷ lệ.
- Lateral RMS cân theo thời gian: sqrt(sum(dt * e_y²) / sum(dt)); thêm RMSE theo số mẫu để so với báo cáo cũ.
- Đỉnh |e_y|, IAE, ISE, ITAE; tỷ lệ thời gian trong `--max-deviation`. Tham số phân tích này không đổi outcome do hệ thống đã ghi.
- Heading RMS, RMS vector gia tốc, RMS Z, đỉnh gia tốc, sampled jerk RMS; đỉnh roll/pitch và RMS angular velocity nếu có.
- Wheel-tracking RMSE khi có cả target và measured; sai số AMCL–GT khi có đủ hai pose.
- RMS base/residual/total torque; RMS torque trung bình hai bánh. Đỉnh torque, torque trái/phải ở mẫu cuối, tích phân bình phương torque và tốc độ thay đổi torque theo mẫu đã ghi.
- Tỷ lệ saturation/slew chỉ từ cờ đo được. Không đoán cờ từ torque ở nhịp RL. Tích phân torque² có đơn vị Nm²s, không phải Joule hay năng lượng điện tiêu thụ.
- Tổng reward và từng thành phần reward; cảnh báo khi định nghĩa reward giữa các run khác nhau, không coi reward normalized và raw là cùng thang đo.

Nếu có `step_duration`, nó là trọng số. Nếu thiếu, dùng chênh timestamp, mẫu đầu không có khoảng thời gian trước đó. Timestamp trùng chỉ được chấp nhận tại mẫu kết thúc và có trọng số 0; không bỏ qua timestamp lùi. `simulation_time` native có gốc episode nên duration bằng timestamp cuối; `time_s`/timestamp ngoài không có gốc episode xác nhận nên duration chỉ là cuối−đầu.

Cửa sổ gồm các sample ở |tọa độ−tâm| <= radius. Chỉ tích phân khoảng có hai đầu sample cùng nằm trong cửa sổ. Không nội suy contact time hoặc gán toàn bộ dt ngoài vùng cho gờ. Cửa sổ chưa đi hết vẫn được xuất và đánh dấu partial; bảng HTML chỉ gộp RMS của cửa sổ đủ hai phía, có mẫu hợp lệ và episode hoàn tất. Nếu các cửa sổ chồng lấn, không cộng chúng như các sự kiện độc lập.

Không có tín hiệu → N/A / ô CSV trống / LaTeX `--`; một episode không đủ để ước lượng SD. `n` khác nhau giữa metric phải được giữ khi đưa số liệu vào paper.

## Tệp đầu ra

- `comparison_summary.csv`: một dòng mỗi nhãn, count/success CI và mean/std/n của mọi chỉ số; đơn vị theo tên metric và JSON.
- `comparison_table.tex`: bảng booktabs gọn, có chuyển lateral error sang cm; thêm `\usepackage{booktabs}` trong paper. Tên phương pháp được escape ký tự LaTeX.
- `benchmark_report.html`: độc lập, nhúng SVG và toàn bộ dữ liệu để chọn nhóm/episode/tín hiệu/trục. Chép riêng HTML vẫn xem được offline; giữ các file cạnh nó nếu muốn bấm liên kết CSV/TEX.
- `episode_metrics.csv`: tất cả episode đã đọc, gồm FAILED/TIME_LIMIT/INCOMPLETE và coverage theo thời gian của các tín hiệu.
- `obstacle_windows.csv`: từng nhóm/episode/tâm, có tọa độ hay không, đã vào/qua tâm/đủ cửa sổ và các metric vùng.
- `benchmark_data.json`: cấu hình phân tích, phiên bản Python/NumPy, metadata/config nguồn, SHA256 log, checkpoint timestep đọc an toàn từ JSON trong zip nếu file còn trên máy, audit và mọi chỉ số.
- `audit.json`: log hỏng, phiên khởi động không có trajectory, startup retries, exception và lượt bị ngắt. Không đổi lỗi hạ tầng thành robot failure.

Bật `--strict-comparability` để từ chối các khác biệt đã phát hiện ở điều kiện môi trường/map hoặc dữ liệu train/smoke/đánh giá không deterministic. Khác reward được cảnh báo thay vì từ chối vì đó có thể là ablation có chủ ý. Không có metadata không đồng nghĩa đã xác nhận cùng điều kiện. Cùng nhãn seed không phải cùng nhiễu Gazebo/AMCL; cùng path map không chứng minh lịch sử map/Nav2/actuator giống nhau.

Không có kiểm định ý nghĩa thống kê hay tự động tuyên bố model tốt nhất. Đây là công cụ tổng hợp và mô tả thí nghiệm, không thay thế thiết kế đối chứng.
