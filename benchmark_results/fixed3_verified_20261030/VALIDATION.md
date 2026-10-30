# Những phần đã và chưa kiểm chứng

133 test toàn bộ đạt; 6 lượt headless đầy đủ (2/controller), cùng reference/config/limits. Ground truth và IMU có coverage 100%; jerk khoảng 99.9%. Checkpoint SAC 50k được xác minh bằng byte và config/metadata; model 50k, 70k cũ giữ nguyên. PID và PI+SAC dùng gain nền khác nhau đúng protocol, PI/SAC giữ Kp=0.30 Ki=0.10 Kd=0.

Nav2 báo SUCCEEDED 6/6 nhưng tiêu chuẩn đầy đủ AMCL+GT đạt 0/6. Xem mission_failure_reasons trong episodes.csv, từng dòng goal/deviation và biểu đồ. Không nới dung sai hay loại các lượt này.

Hiệu chỉnh thử 6 ứng viên, tối đa 2 finalist, không ứng viên nào đạt toàn bộ tiêu chí; profile không đổi. Hồ sơ và mọi log được giữ tại rl_training/runs/pid_tuning_20261030_measured_D.

HTML offline và tương tác chọn lượt đã kiểm tra bằng Chrome headless, cả 5 biểu đồ có polyline thực. Bảng TeX kiểm tra booktabs/cột/thống kê, chưa biên dịch vì chưa có pdflatex. Chưa chạy đánh giá 7–10 lượt mỗi controller, chưa thử robot thật hoặc GUI sau thay đổi này. Không gọi hệ thống trơn tru hoặc PID tối ưu.

VALIDATION.json liệt kê cả các smoke bị lỗi/gián đoạn trước đó; chúng không bị xóa hay đổi thành thất bại điều khiển trong bộ evaluation.
