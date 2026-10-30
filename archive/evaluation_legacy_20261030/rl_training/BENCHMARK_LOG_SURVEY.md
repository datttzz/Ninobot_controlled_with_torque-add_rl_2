# Khảo sát log trước khi xây dựng benchmark đa mô hình

Nguồn đã đọc: `ros_env.py`, `train.py`, `features.py`, `compare_runs.py`, `evaluation_report.py`, `plot_run.py`, `config/sac.yaml`, `src/nino_navigation/config/nav2.yaml`, `src/nino_description/worlds/long_hall.sdf` và các run PI/50k/70k thực trên máy.

## Định dạng thực tế

- Mỗi run có `config.yaml`, `metadata.json`, `monitor.csv`, `result.json` (nếu đã có episode hoàn tất).
- Mỗi `episode_*` có `steps.jsonl`, `simulation.log`, `navigation.log`. Có thể có thư mục khởi động lỗi chưa có `steps.jsonl`.
- `monitor.csv` là CSV tổng hợp Stable Baselines3, header `r,l,t`: tổng reward, số step, thời gian tường tính từ lúc mở Monitor. Đây không phải timestamp mẫu cảm biến hay thời gian hoàn thành mô phỏng.
- Mẫu JSONL được ghi ở ranh giới step RL (~0,1 giây), không phải toàn bộ IMU/control loop. Các topic được lấy gần nhất, không nội suy đồng bộ.

| Đại lượng | Key thực trong steps.jsonl | Ghi chú |
|---|---|---|
| Thời gian | `simulation_time`, `step_duration` | Giây; simulation_time từ đầu episode |
| Lỗi ngang/hướng | `cross_track`, `heading_error` | Mét/radian, tính từ TF map→base_footprint của AMCL |
| Tiến triển | `progress`, `remaining` | Mét dọc đường tham chiếu; không phải tọa độ X |
| Pose AMCL X/Y/yaw | Không lưu | Dùng để tính feature nhưng không nằm trong info được ghi |
| Ground truth pose | Không lưu | Không thể đo sai số định vị AMCL–GT từ log cũ |
| Gia tốc | `acceleration` = [ax,ay,az] | m/s² đã trừ trọng lực bằng orientation IMU, xem imu_features() |
| Góc thân xe | `roll`, `pitch` | Radian; đây là góc, không phải angular velocity |
| Angular velocity IMU | Không lưu | w.x/y/z có trong observation nhưng không ghi log |
| Vận tốc bánh và target | Không lưu | Có trong observation nhưng không ghi log; không khôi phục bằng giả định |
| Mô-men nền | `base_torque` = [left,right] | Lệnh PI đã clip trước khi cộng residual |
| Residual | `residual_applied`, `commanded_residual` | Nm; applied là phần được chấp nhận trước limiter cuối |
| Mô-men tổng | `total_torque` = [left,right] | Lệnh cuối được báo trên /wheel_torque_applied; không phải cảm biến mô-men |
| Cờ saturation/slew | Không lưu | Không suy ra tỷ lệ clamp thật từ torque ở nhịp RL |
| Reward | `reward_terms` | progress, cross_track, heading, vibration, effort, smoothness, time, terminal; tổng là reward mỗi step |
| Kết quả | `outcome`, `reason` | SUCCEEDED/FAILED/TIME_LIMIT ở mẫu kết thúc; thiếu outcome là chưa hoàn tất |
| Đường | `reference_geometry_sha256`, `path_sha256`, `planning_requests` | Hash hình học dùng để kiểm tra đường cố định trong episode |

Các script cũ chỉ so sánh một baseline PI với một policy. Công cụ mới không suy ra loại controller từ tên thư mục hay giới hạn số nhóm. PID là nhãn dữ liệu nhập vào; không thêm controller PID vào robot.

## Cấu hình ảnh hưởng phép so sánh

SAC: step 0,1s, episode tối đa 90s, ngưỡng lệch đường 0,20m, residual tối đa 0,30Nm; goal (11,0). Reward có nhiều thành phần, nên reward tổng giữa cấu hình reward khác nhau không thể dùng trực tiếp để xếp hạng.

Nav2: vòng controller 20Hz, max_vel_x=0,25m/s, max_vel_theta=0,6rad/s; goal tolerance XY/yaw 0,25m/rad. Có velocity_smoother và collision_monitor. Metadata cũ không chụp toàn bộ cấu hình Nav2/actuator tại thời điểm chạy, nên không thể chứng minh chúng giống nhau chỉ bằng cùng đường dẫn map.

## Gờ/dây trong long_hall hiện tại

Tọa độ dưới đây là tâm collision trong world. Dây nằm nghiêng; vị trí giao với đường chạy phụ thuộc Y. Bản đồ occupancy hall_01.pgm không cung cấp nhãn thời điểm tiếp xúc bánh–gờ.

| Collision | X (m) | Y (m) | Bán kính (m) | Yaw (rad) |
|---|---:|---:|---:|---:|
| cable_01_collision | 0.5 | 0 | 0.005 | 0.139626 |
| cable_02_collision | 1 | 0 | 0.00875 | -0.209440 |
| cable_03_collision | 1.5 | 0 | 0.00625 | 0.314159 |
| cable_04_collision | 2.1667 | 0 | 0.0125 | -0.104720 |
| cable_05_collision | 2.6667 | 0 | 0.0075 | 0.383972 |
| cable_06_collision | 3.3333 | 0 | 0.01125 | -0.279253 |
| cable_07_collision | 4 | 0 | 0.005625 | 0.174533 |
| cable_08_collision | 4.8333 | 0 | 0.01375 | -0.349066 |
| cable_09_collision | 5.6667 | 0 | 0.009375 | 0.244346 |
| cable_10_collision | 6.5 | 0 | 0.006875 | -0.139626 |
| cable_11_collision | 7.3333 | 0 | 0.011875 | 0.418879 |
| cable_12_collision | 8 | 0 | 0.008125 | -0.244346 |
| cable_13_collision | 8.6667 | 0 | 0.013125 | 0.104720 |
| cable_14_collision | 9.5 | 0 | 0.01 | -0.383972 |

Ví dụ `--obstacle-x 3 6 9` là các cửa sổ do người dùng chỉ định, không phải tâm dây đúng của world này. Tính cửa sổ theo X cần pose AMCL/GT X thực trong log; `progress` không được tự đổi thành X. Có thể chủ động chọn `--obstacle-axis progress` để phân tích vùng theo tiến triển dọc đường, và báo cáo sẽ ghi rõ cách xấp xỉ này. Các cửa sổ không phải bằng chứng robot đã chạm hoặc vượt thành công một gờ vật lý.
