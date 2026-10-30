# Mã đánh giá cũ được lưu nguyên bản

Không xóa checkpoint, kết quả, report, log hay replay buffer. Workspace này không có Git nên không xóa bản code lưu trữ. MANIFEST.json ghi SHA256 và đường dẫn gốc. Có thể khôi phục bằng cách chép file về đường dẫn original sau khi sao lưu code mới; không chạy script trong archive bằng mặc định cũ.

Luồng đánh giá hiện hành: run_ablation_suite.py --sac-checkpoint ...; phân tích: benchmark_pi_pid_sac.py --pi-dir ... --pid-dir ... --sac-dir ...
