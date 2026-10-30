# Kiểm tra sáu episode đã ghi

Giữ nguyên các log, model, Nav2 và ngưỡng. Chỉ tính lại, không chạy SAC.

| Controller/lượt | NAV | AMCL xy / yaw | GT xy / yaw | Max ngang AMCL / GT | AMCL / GT / Full | Nguyên nhân |
|---|---|---|---|---|---|---|
| PI/00001 | SUCCEEDED | 0.227581 m / 0.187879 rad | 0.289214 m / 0.200789 rad | 0.143319 / 0.151080 m | True / False / False | gt_goal_xy_m=0.289214 > 0.25 |
| PI/00002 | SUCCEEDED | 0.218191 m / 0.217599 rad | 0.309023 m / 0.209526 rad | 0.188914 / 0.226928 m | True / False / False | gt_goal_xy_m=0.309023 > 0.25; gt_max_m=0.226928 > 0.2 |
| PID/00001 | SUCCEEDED | 0.195871 m / 0.175175 rad | 1.036375 m / 0.193391 rad | 0.109463 / 0.140596 m | True / False / False | gt_goal_xy_m=1.03638 > 0.25 |
| PID/00002 | SUCCEEDED | 0.231662 m / 0.173133 rad | 0.333788 m / 0.173276 rad | 0.112787 / 0.157115 m | True / False / False | gt_goal_xy_m=0.333788 > 0.25 |
| PI_SAC/00001 | SUCCEEDED | 0.226185 m / 0.080754 rad | 0.283940 m / 0.092475 rad | 0.084198 / 0.106444 m | True / False / False | gt_goal_xy_m=0.28394 > 0.25 |
| PI_SAC/00002 | SUCCEEDED | 0.083799 m / 0.247992 rad | 0.150666 m / 0.256693 rad | 0.084343 / 0.102091 m | True / False / False | gt_goal_yaw_rad=0.256693 > 0.25 |

Tất cả thời điểm max, timestamp nguồn và timestamp terminal có trong six_episodes.csv và audit.json.
Đích được kiểm tra tại terminal sau STOPPING; không phải đúng chu kỳ FollowPath báo reached_goal.
Max tính từ lúc logger bắt đầu sau FOLLOWING đến terminal; không chứa reset. Các max cũ đều xuất hiện trước vùng dừng cuối, xem timestamp.
AMCL ở đây là TF map→base_footprint kết hợp AMCL và odometry, không phải message /amcl_pose đơn lẻ.
Dữ liệu pose cuối không đổi giữa mẫu 50 Hz cuối và terminal; độ lệch GT không do lấy nhầm mẫu sau reset.
Log cũ không có quaternion gốc, state từng mẫu hoặc timestamp chuẩn Nav2 reached_goal; không thể khôi phục các trường đó từ yaw. Logger mới bổ sung thụ động các bằng chứng này.
Không fit GT vào AMCL và không sửa pose GT. Identity map/world là hợp đồng tọa độ danh định; sai số bản đồ/định vị được giữ là sai số thật của hệ thống.
