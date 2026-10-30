# Hiệu chỉnh PID sau khi kiểm tra phép đo

Kết thúc sau 11.15 phút; chỉ PID, residual tắt; không train/nạp/evaluate checkpoint SAC. Một Gazebo headless mỗi lần.

Sáu ứng viên cố định trước khi chạy; không ứng viên nào đạt toàn bộ hard constraints. Không có finalist đủ điều kiện; không thêm gain sau khi xem kết quả. Dữ liệu cũ tái sử dụng làm bằng chứng loại trong prior_data_review.json (plant/controller/Nav2/limits tương thích).

**Chưa có bộ PID đủ điều kiện vận hành. Profile hiện hành không đổi.**

## Ứng viên khóa để kiểm tra riêng

Kp=0.35, Ki=0.10, Kd=0.0002, tau_D=0.08 s. Profile: pid_frozen_candidate.yaml. Đã ghi hash/freeze time trước xác minh; FREEZE_EVIDENCE.json và FINAL_VALIDATION.json kiểm tra không thay đổi.

D_raw=-Kd*(omega[k]-omega[k-1])/dt; alpha=tau_D/(tau_D+dt); D=alpha*D_prev+(1-alpha)*D_raw. dt lấy timestamp joint_states thực; không lấy đạo hàm setpoint. Ví dụ dt=0.001 s thì alpha≈0.987654; đây không phải hằng số gán cố định. Không sửa code controller trong đợt này: các xử lý sample trùng/cũ, dt lỗi, I/D reset và limiter đã có và test lại đạt.

Chọn ứng viên dự phòng theo protocol đã ghi trước: số full pass, số điều kiện loại, rồi lexicographic wheel RMSE → GT RMSE → RMS vận tốc lệch ngang đã lọc 5 Hz → RMS góc roll. Không có điểm tổng hợp/trọng số. Đây là ứng viên chưa đạt, không được gọi là PID đạt chuẩn hay tối ưu. Sáu lượt đầu chỉ bộ 0.35/0.10/0.0002/0.08 có wheel RMSE thấp nhất trong các lượt cùng số điều kiện loại. Ba lượt xác minh không dùng để đổi gain.

## Tất cả lượt (không bỏ thất bại)

| Lượt | NAV / Full | GT xy / yaw cuối | Max ngang GT | Wheel RMSE | GT RMSE | Slew / sat |
|---|---|---|---|---|---|---|
| screen_p025_i010_d0002_t050 #1 | True / False | 0.316590 m / 0.003782 rad | 0.148467 m | 1.305637 rad/s | 0.064796 m | 21.209% / 0.002% |
| screen_p030_i010_d0002_t080 #1 | True / False | 0.273337 m / 0.096012 rad | 0.117612 m | 1.228051 rad/s | 0.058472 m | 23.571% / 0.012% |
| screen_p030_i010_d0005_t050 #1 | True / False | 0.297260 m / 0.180796 rad | 0.131296 m | 1.247407 rad/s | 0.066457 m | 25.638% / 0.036% |
| screen_p035_i010_d0002_t080 #1 | True / False | 0.289498 m / 0.144869 rad | 0.133901 m | 1.192316 rad/s | 0.059777 m | 26.645% / 0.360% |
| screen_p025_i012_d0005_t080 #1 | True / False | 0.131862 m / 0.275446 rad | 0.224778 m | 1.262978 rad/s | 0.081956 m | 18.244% / 0.004% |
| screen_p040_i010_d0005_t080 #1 | True / False | 0.321984 m / 0.236920 rad | 0.199358 m | 1.332178 rad/s | 0.077877 m | 32.462% / 1.504% |
| verification #1 | True / False | 0.296681 m / 0.130279 rad | 0.104269 m | 1.215895 rad/s | 0.058221 m | 27.650% / 0.356% |
| verification #2 | True / False | 0.099662 m / 0.250550 rad | 0.236077 m | 1.145104 rad/s | 0.083761 m | 23.902% / 0.277% |
| verification #3 | True / False | 0.324184 m / 0.216777 rad | 0.244180 m | 1.247165 rad/s | 0.096125 m | 27.694% / 0.350% |

## Ba lượt xác minh giữ riêng

NAV 3/3; full 0/3. Xem verification_3_episodes.csv: giữ đủ lỗi vị trí/hướng/hành lang. Chưa đạt tiêu chí nên không triển khai profile vào rl_training/config/pid.yaml.

| Chỉ số giữa 3 lượt | Mean ± sample Std |
|---|---|
| wheel_rmse_rad_s | 1.202721 ± 0.052290 |
| gt_rmse_m | 0.079369 ± 0.019330 |
| lateral_rate_rms_m_s | 0.033822 ± 0.002883 |
| roll_angle_rms_rad | 0.018683 ± 0.001316 |
| roll_rate_rms_rad_s | 0.147633 ± 0.010375 |
| torque_rms_nm | 0.417852 ± 0.013968 |
| saturation_pct | 0.327977 ± 0.044118 |
| slew_limit_pct | 26.415361 ± 2.176329 |
| average_speed_m_s | 0.209490 ± 0.016951 |

Thời gian hoàn thành đầy đủ: N/A vì 0/3 full pass. duration_s trong CSV chỉ là thời gian tới terminal, không giả làm hoàn thành nhiệm vụ.
obstacle_windows.csv ghi 14 gờ thực của SDF, cửa sổ ±0.5 m theo tiến độ s, độ sụt tốc so với vùng trước gờ và full/partial window. Không giảm tốc Nav2. Điểm GT world y=0 và đích world (11,0) giữ nguyên theo giai đoạn A.
Tỷ lệ slew không bị yêu cầu bằng 0. Tất cả lệnh torque sau limiter <=2 Nm, slew 10 Nm/s; target bánh <=12 rad/s; physical emergency 24 rad/s; watchdog/anti-windup/collision monitor giữ nguyên. Counter limiter lấy chênh chu kỳ 1kHz; wheel/GT/IMU lấy mẫu logger 50Hz.

## Hạn chế và blocker còn lại

Phép đo đủ dùng cho mission hall_01 đã định nghĩa, nhưng hệ odometry/localization + Nav2 hiện tại vẫn dừng thành công theo ước lượng khi robot chưa đạt GT; một số lượt còn vượt hành lang GT. Chưa chứng minh PID có thể giải quyết giới hạn này. Không hiệu chỉnh AMCL/Nav2 hoặc thay đích/ngưỡng trong nhiệm vụ này.
Ngân sách hữu hạn, sáu gain và ba lượt xác minh không chứng minh tối ưu toàn cục hoặc hơn PI. GUI chỉ kiểm tra dry-run cấu hình, không chạy GUI trong đợt này. Mọi lỗi điều khiển được giữ; không có trial lỗi hạ tầng trong đợt mới. 38 test liên quan và một integration A đã chạy; không chạy lại toàn suite.

Thay đổi nguồn: thêm mission_measurement.py, audit_mission_measurements.py, calibrate_pid_measured.py và test; ros_env.py chỉ thêm bằng chứng pose/frame/quaternion/state/clock; experiment_guard.py nhận diện thêm job tuning mới. Không sửa Nav2/URDF/dynamics/controller code hay profile PI. Bản trước sửa lưu archive/mission_pid_audit_20261030.

## Lệnh chạy riêng PID ứng viên với GUI

Chỉ dùng để kiểm tra ứng viên chưa đạt, không coi là profile vận hành đã xác nhận. Đóng các job/Gazebo cũ trước; runner sẽ từ chối chạy trùng.

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source rl_training/.venv/bin/activate
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_pid_gui
python run_ablation_suite.py --modes PID --episodes 1 --gui --seed 7101 \
  --pid-config rl_training/runs/pid_measured_tuning_20261030/pid_frozen_candidate.yaml \
  --output-root "rl_training/runs/pid_gui_$(date +%Y%m%d_%H%M%S)"
```
