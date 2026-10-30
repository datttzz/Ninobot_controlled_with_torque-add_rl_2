# Giai đoạn A — Kết luận phép đo

## Kết quả

Sáu log gốc được tính lại, không thay byte dữ liệu: NAV 6/6, AMCL độc lập 6/6, GT độc lập 0/6, full 0/6. Bảng sáu lượt nằm ở SIX_EPISODES.md và six_episodes.csv. Timestamp terminal và timestamp từng max có trong CSV; audit.json lưu kiểm tra nguồn, độ lệch thời gian, hash raw log và lỗi số học.

Năm lượt dừng ngoài vùng đích GT (0.289214, 0.309023, 1.036375, 0.333788, 0.283940 m); lượt cuối nằm trong vùng xy nhưng yaw GT 0.256693 rad >0.25. PI lượt 2 còn có max ngang GT 0.226928 m >0.20. Không bỏ các thất bại này. Sai số hiển thị được làm tròn, quyết định dùng float64 <= trực tiếp, không thêm epsilon. Chênh lệch sát ngưỡng vẫn được giữ đúng kết quả.

## Frame và điểm vật lý

- Gazebo OdometryPublisher lấy `worldPose(model.Entity()) * offset`, không lấy AMCL và không tích phân encoder để tạo pose. `<robot_base_frame>` chỉ đặt nhãn, vì vậy đã kiểm tra model origin thay vì tin nhãn. Nguồn: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc (UpdateOdometry).
- Xacro và SDF đã chuyển đổi được lưu expanded_robot.urdf/sdf: model nino gốc trùng link gốc base_footprint; base_link chỉ lệch z=0.0625 m qua fixed joint. Không có lệch x/y hay quay ở fixed joint. GT chính là điểm model/base_footprint vật lý. Không sửa URDF.
- Reference `map` là (0,0,0) -> (11,0,0). Requested goal và endpoint cùng (11,0), yaw=0. Trong phép đo này mục tiêu vật lý được định nghĩa độc lập là world (11,0), đường world y=0 (giữa tường ±2 m của SDF). Registration identity là hợp đồng tọa độ danh định của nhiệm vụ/spawn/mapping ban đầu, KHÔNG phải phép căn chỉnh lại theo AMCL từng lượt, cũng không phải TF map/world được ước lượng online.
- Kiểm tra raster map độ phân giải 0.05 m: các dải tường tại x=-1.943/12.057 và y xấp xỉ -2.012..-1.962/1.988..2.038 tương ứng hình học hall world x=-2/12,y=±2 trong độ phân giải/sai số dựng bản đồ. Không có bằng chứng map quay/đổi gốc lớn. Sai số raster/SLAM không bị dùng để dịch GT cho pass. Không tuyên bố đăng ký đo đạc map/world chính xác tuyệt đối; không tổng quát hóa identity sang map khác.

## Vì sao Nav2 thành công không đảm bảo GT đạt

Nav2 chỉ biết pose ước lượng map→base_footprint. `AMCL` trong log là TF hợp AMCL + wheel odometry, không phải riêng /amcl_pose. Goal checker hiện có stateful=true: sau khi đã vào vùng xy nó có thể chỉ xét yaw; nguồn https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/simple_goal_checker.cpp . Tuy nhiên cả sáu AMCL cuối vẫn đạt cả xy/yaw, nên stateful không phải lời giải thích thay cho GT ở đây.

TF–GT chênh x từ 0.001..0.009 m đầu lượt thành 0.056..0.095 m cuối ở năm lượt; PID lượt 1 tăng đến 0.840261 m. Đây không phải một offset frame tĩnh. Tích phân vận tốc bánh với bán kính 0.0625 m cho PID lượt này khoảng 11.503 m trong khi quãng đường GT khoảng 10.126 m (chỉ bằng chứng chênh động học, không phải bộ ước lượng trượt chính xác vì lấy mẫu 50 Hz). Phù hợp với odometry tích lũy sai số/trượt qua gờ và định vị dọc hành lang chưa bù đủ; không thể khẳng định riêng tỷ lệ đóng góp AMCL/encoder khi log cũ không ghi từng transform thành phần hoặc laser scan. Xem drift_evidence.json. Wheel-speed PID không có quyền sửa trực tiếp sai số định vị này.

## Thời điểm và tính số học

- Logger bắt đầu sau FOLLOWING, không ghi reset/khởi tạo. Max ngang xuất hiện 3–34 s trước terminal, không nằm trong dừng cuối; không có trường hợp reset làm max vượt.
- EpisodeManager nhận FollowPath success, chuyển STOPPING, gửi zero, xác nhận vận tốc thấp liên tục 0.2 s rồi trả action ngoài SUCCEEDED. Full-pass xét terminal đã dừng, không cố giả làm chu kỳ nội bộ Nav2 reached_goal; giữ nguyên định nghĩa terminal này cho tuning.
- Hai pose cuối (mẫu 50 Hz cuối và terminal) đều không đổi x/y/yaw. Timestamp AMCL/GT có lệch lấy mẫu tối đa 0.037 s, không lùi; bản thân pose không được ép đồng bộ bằng fit. Không cần nội suy để đảo kết quả các pose dừng này. Timestamp nguồn được giữ riêng, không giả rằng cùng clock stamp.
- Trên đường hiện tại mọi pose nằm trong đoạn X=0..11; công thức hình chiếu cho |e_y| khớp |Y|, sai khác lớn nhất 2.78e-17 m. Góc được wrap bằng atan2(sin,cos). Đường cong/đoạn ngoài endpoint không được suy ra từ phép kiểm tra đường thẳng này.
- Log cũ thiếu raw quaternion, child frame và phase từng mẫu; không tạo lại những trường thiếu từ yaw. Logger mới thêm bằng chứng thụ động, không thay observation/reward/controller. Một PID integration headless độc lập ghi FOLLOWING/STOPPING/SUCCEEDED, cả hai child frame base_footprint; quaternion chuẩn tới 2.69e-14, yaw tính lại sai khác 0; age AMCL<=0.022 s, GT<=0.012 s. Xem INTEGRATION_EVIDENCE.json.

## Sửa đổi tối thiểu và quyết định

Không cần sửa công thức/ngưỡng của báo cáo cũ; không ghi đè báo cáo. Thêm mission_measurement.py để tách NAV/AMCL/GT/full; audit_mission_measurements.py để tính bảng có bằng chứng; ros_env.py chỉ thêm raw frame/quaternion/clock/state vào log mới. 30 test liên quan đạt (mission, metrics, derivative, ROS PID).

Đủ tin cậy để tuning trên đúng hall_01 và nhiệm vụ world y=0/goal (11,0), dùng GT độc lập và wheel velocities/IMU đã kiểm chứng. Định vị dọc hành lang vẫn là giới hạn hiệu năng thực, không phải lý do để nới ngưỡng hay căn chỉnh GT. Không có blocker về phép đo cho giai đoạn B. Đăng ký map khác hoặc phân rã định lượng AMCL/encoder là ngoài phạm vi, chưa xác minh.
