# Dữ liệu được đồng bộ lên Git

Repository riêng tư: https://github.com/datttzz/Ninobot_controlled_with_torque-add_rl_2

- Lưu mã nguồn, cấu hình, bản đồ, mesh robot, tài liệu và code đã archive.
- Lưu các báo cáo trong `benchmark_results/`.
- Lưu chọn lọc báo cáo, hình, bảng số liệu và profile ứng viên trong ba thư mục
  `rl_training/runs/mission_audit_20261030`, `pid_measured_tuning_20261030`
  và `localization_diagnosis_20261030`. Dùng `git ls-files rl_training/runs`
  để xem chính xác những file đã được lưu.
- Không đưa môi trường Python, build/install/log, cơ sở dữ liệu VS Code,
  rosbag, checkpoint SAC/replay buffer và telemetry thô vào Git. Các file đó
  vẫn giữ nguyên trên laptop; clone repo không phải bản sao đầy đủ dữ liệu train.
- Các checkpoint và raw log mà báo cáo tham chiếu cần được sao lưu riêng nếu
  muốn tiếp tục train hoặc tính lại phân tích từ đầu trên máy khác.

`rl_training/runs/` được ignore mặc định để tránh vô tình đẩy dữ liệu lớn.
Những báo cáo chọn lọc đã được thêm có chủ ý vẫn được Git theo dõi. Báo cáo
giữ nguyên kết luận và hạn chế của từng thí nghiệm; việc đưa lên Git không
đồng nghĩa PID đã đạt chuẩn hoặc SAC đã được xác nhận cải thiện.
