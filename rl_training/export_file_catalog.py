#!/usr/bin/env python3
"""Xuất danh mục file HTML offline; chỉ đọc repo, không import/chạy code robot."""
import argparse
import ast
from collections import Counter
from datetime import datetime
import json
import os
from pathlib import Path
import stat
from zoneinfo import ZoneInfo


# Giải thích theo đường dẫn cụ thể; file archive không nhận mô tả hành vi hiện hành.
# Mỗi dòng: đường dẫn | nghĩa tên | công việc | đầu vào | đầu ra / cách dùng / giới hạn.
SPEC = r"""
README.md|Đọc tôi trước|Giới thiệu dự án và các bước sử dụng tổng quát.|Người đọc cần nắm cấu trúc repo.|Tài liệu; đối chiếu hướng dẫn laptop và code hiện hành khi gặp thông tin lịch sử.
NAVIGATION.md|Điều hướng|Tài liệu về Nav2, định vị, lập đường và thực thi nhiệm vụ.|Bản đồ, hệ thống ROS và các node điều hướng.|Hướng dẫn; không phải script điều khiển.
HUONG_DAN_CHAY_TREN_LAPTOP.md|Hướng dẫn chạy trên laptop|Hướng dẫn chuẩn bị môi trường và các lệnh theo terminal.|Laptop ROS 2 Jazzy và môi trường Python đã chuẩn bị.|Các bước chạy mô phỏng, train và thực nghiệm.
AGENTS.md|Quy tắc cho tác nhân sửa code|Nêu phạm vi, môi trường và quy tắc làm việc trong repo.|Công việc sửa code.|Tài liệu; không tham gia điều khiển xe lúc chạy.
GIT_DATA_POLICY.md|Chính sách dữ liệu Git|Giải thích cách quản lý mã nguồn, dữ liệu và kết quả trong Git.|Người quản lý repo.|Hướng dẫn lưu trữ và chia sẻ dữ liệu.
.gitignore|Danh sách Git bỏ qua|Quy định đường dẫn Git không tự thêm vào commit.|Các mẫu tên file/thư mục.|Không xóa dữ liệu trên máy; không tự bỏ theo dõi file đã commit.
pytest.ini|Cấu hình pytest|Chỉ định thư mục tìm test và các thư mục cần bỏ qua khi thu thập test.|Lệnh pytest.|Cấu hình kiểm thử; không chạy mô phỏng khi chỉ đọc file.
compose.yaml|Cấu hình Docker Compose|Khai báo container phát triển, GPU, mạng và các thư mục gắn vào container.|Docker và các biến môi trường hệ thống.|Môi trường Docker; có cấu hình WSLg, không đồng nghĩa laptop Ubuntu phải dùng Docker.
.devcontainer/Dockerfile|Công thức tạo image phát triển|Cài ROS Jazzy, Gazebo, Nav2, RViz và công cụ phát triển vào image Docker.|Image ROS nền và apt packages.|Image container sau khi build; không phải firmware STM32.
run_ablation_suite.py|Chạy bộ thực nghiệm đối chứng|Điểm vào ở thư mục gốc; import main từ rl_training.ablation_suite rồi gọi nó.|CLI của ablation_suite.py.|Không chứa toàn bộ logic runner; mở ablation_suite.py để xem phần thực thi.
benchmark_pi_pid_sac.py|Đối sánh PI, PID và PI + SAC|Đọc ba nhóm log, tính chỉ số AMCL/IMU/encoder/controller và dựng báo cáo offline.|--pi-dir, --pid-dir, --sac-dir, --output-dir; tùy chọn --obstacle-x.|summary.csv, episodes.csv, obstacle_windows.csv, obstacle_summary.csv, comparison_table.tex, report_data.json, audit.json, report_PI_PID_SAC.html. Không train và không mở ROS.
rl_training/train.py|Huấn luyện|Điểm chạy SAC: train, smoke, baseline residual bằng 0, hoặc evaluate deterministic.|--mode, --map, --config; train dùng --steps; resume cần cặp model/replay.|Tạo thư mục rl_training/runs theo thời gian, lưu config/metadata/model/log. --steps là tổng đích; --gui mở Gazebo. Smoke chỉ kiểm tra tích hợp, không chứng minh policy tốt.
rl_training/long_run.py|Theo dõi một lần train dài|Tìm cặp model/replay trong run cũ, đọc num_timesteps, rồi gọi train.py ở tiến trình con để resume. Ghi trạng thái và chuyển yêu cầu ngắt sang tiến trình train.|Bắt buộc --resume-train-run, --map, --output; tùy chọn --steps, --checkpoint-every, --device.|Thư mục --output chứa train.log và status.json; run train thật nằm ở đường dẫn Run: do train.py tạo. Ưu tiên cặp model/replay hoàn chỉnh trước khi dò checkpoint. Mặc định --steps=50000000, phải chọn đích rõ ràng. Không có --gui; không tự chạy benchmark ba controller.
rl_training/ros_env.py|Môi trường ROS cho RL|NinoResidualEnv nối Gymnasium với một phiên Gazebo/Nav2; reset các tiến trình do nó sở hữu, nhận cảm biến, tạo observation/reward và gửi residual.|Config, bản đồ, thư mục run, dữ liệu IMU/JointState/odom/TF map-base và chẩn đoán controller.|Observation 23 phần tử, reward, trạng thái episode; steps.jsonl và telemetry.jsonl khi bật luồng benchmark. Code hiện hành dùng AMCL/IMU/encoder, không cần GT Gazebo.
rl_training/features.py|Đặc trưng và reward|Tính lỗi ngang, lỗi hướng, tiến độ/đường còn lại; khử thành phần trọng lực IMU bằng quaternion; tính từng thành phần reward.|Pose, đường tham chiếu, IMU, action, dt và trọng số reward.|Các giá trị số dùng trong môi trường; không tự subscribe ROS hoặc train mạng.
rl_training/runtime.py|Thiết bị thực thi|Chọn CPU/CUDA và xác minh actor, critic, phép forward dùng thiết bị yêu cầu.|Yêu cầu --device và model SAC.|Thông tin/kiểm tra thiết bị; lỗi rõ khi CUDA không dùng được.
rl_training/startup.py|Khởi động|Thử lại có giới hạn với lỗi hạ tầng trước khi gửi goal; tách lỗi startup khỏi thất bại điều khiển.|Thao tác khởi động, số lần thử và hàm dọn phiên.|Kết quả khởi động hoặc StartupFault; không dùng để che thất bại sau khi nhiệm vụ đã bắt đầu.
rl_training/episode_outcome.py|Kết quả một lượt|Định nghĩa lỗi hạ tầng và phân loại trạng thái điều hướng.|Trạng thái nhiệm vụ và thông tin lỗi.|Giữ lỗi hạ tầng ngoài luồng reward/replay của thất bại nhiệm vụ.
rl_training/ablation_suite.py|Bộ thực nghiệm đối chứng|Chuẩn bị và chạy tuần tự PI, PID, PI + SAC; kiểm tra checkpoint/config, đường tham chiếu và job đang chạy; ghi bằng chứng cấu hình.|CLI runner, một --sac-checkpoint khi chạy SAC, profile PID, map/config, số episode.|Các nhóm run, metadata/log/cấu hình snapshot. Giữ PI nền .30/.10/Kd0; phân tích báo cáo bằng script khác.
rl_training/experiment_guard.py|Giám sát thực nghiệm|Phát hiện job liên quan và kiểm tra điều kiện dừng tài nguyên/thời hạn.|Thông tin tiến trình, tài nguyên và cấu hình giới hạn.|ResourceStop hoặc log tài nguyên; không tự kill tiến trình không thuộc phiên đang quản lý.
rl_training/mission_measurement.py|Đo điều kiện nhiệm vụ|Kiểm tra đích và hành lang theo định vị, giữ riêng Nav2 outcome và AMCL task pass.|Pose cuối, goal, max deviation và dữ liệu đủ/thiếu.|Điều kiện pass/fail và bằng chứng nguồn pose; không coi AMCL là vị trí thực tuyệt đối.
rl_training/pose_measurements.py|Phép đo vị trí và hướng|Xử lý timestamp nguồn, mẫu trùng, coverage, nội suy pose terminal và tín hiệu.|Các dòng telemetry, tên nguồn và giới hạn khoảng trống dữ liệu.|Pose/trọng số/coverage hoặc lý do thiếu; không ngoại suy terminal và không bắc qua gap quá dài.
rl_training/localization_math.py|Toán định vị|Quấn góc, hợp/nghịch đảo biến đổi SE(2), nội suy pose với góc yaw.|Pose phẳng và timestamp.|Kết quả toán học; không sinh nguồn định vị và không tự làm AMCL trùng GT.
rl_training/audit_mission_measurements.py|Kiểm tra phép đo nhiệm vụ|Đọc lại log AMCL/IMU/encoder để audit điều kiện và dữ liệu đo.|Log episode đã có.|Kết quả audit; không mở ROS và không sửa dữ liệu gốc.
rl_training/tune_pid.py|Hiệu chỉnh PID|Điểm vào hiện hành chuyển sang quy trình có ngân sách trong calibrate_pid_measured.main.|CLI được chuyển đến calibrate_pid_measured.py.|Không train SAC. Các helper/candidate cũ còn trong file không có nghĩa main hiện hành chạy một đợt khảo sát riêng.
rl_training/calibrate_pid_measured.py|Hiệu chỉnh PID dựa trên dữ liệu đo|Khóa protocol, tái phân tích log PID cũ tương thích, chọn/khóa ứng viên rồi giới hạn hai lượt xác minh mới.|--output-root, --budget-minutes tối đa 30; dữ liệu AMCL/IMU/encoder và snapshot cấu hình.|Protocol, state, log, profile ứng viên và kết quả xác minh. Chỉ cập nhật profile vận hành khi đáp ứng điều kiện; không bảo đảm tối ưu toàn cục.
rl_training/pid_tuning_data.py|Dữ liệu hiệu chỉnh PID|Kiểm tra tương thích map/Nav2/giới hạn/tham chiếu; tính đo lường, lý do loại và thứ tự chọn gain.|Log/cấu hình ứng viên lịch sử hoặc mới.|compatibility, measurement, rejection_reasons, ranking, combine. Không dùng nhãn GT pass cũ để loại ứng viên.
rl_training/plot_run.py|Vẽ kết quả một lượt|Đọc log episode và xuất hình lỗi đường, IMU và mô-men.|Đường dẫn log episode và tùy chọn CLI.|Hình PNG; không tự chạy robot hoặc sửa reward.
rl_training/plotting_compat.py|Tương thích thư viện vẽ|Sửa namespace mpl_toolkits khi môi trường kế thừa package Ubuntu làm che toolkit của Matplotlib đang chọn.|Môi trường Python/Matplotlib.|Điều chỉnh đường dẫn namespace trước import; không cài lại thư viện và không giấu cảnh báo bằng cách vô hiệu hóa chúng.
rl_training/dashboard.py|Bảng theo dõi|Máy chủ dashboard cục bộ chỉ đọc trạng thái job và log.|Đường dẫn job/run và truy cập trình duyệt.|Trang theo dõi; không phải node ROS, không import Torch để train, không cung cấp nút điều khiển training.
rl_training/demo_server.py|Máy chủ trình diễn|Phục vụ trang demo riêng và điều phối phiên demo SAC có GUI.|Model/config và lệnh từ trang demo.|Phiên demo và dữ liệu giao diện; khác dashboard theo dõi chỉ đọc.
rl_training/dashboard/index.html|Trang chính dashboard|Giao diện trình duyệt của bảng theo dõi.|Dữ liệu do dashboard.py cung cấp.|Hiển thị trạng thái; mở file trực tiếp không thay thế máy chủ API khi cần dữ liệu live.
rl_training/dashboard/demo.html|Trang trình diễn|Giao diện của demo_server.py.|Dữ liệu và thao tác của phiên demo.|Trang demo; không phải báo cáo kết quả khoa học.
rl_training/requirements.txt|Các thư viện cần cài|Khai báo dependency Python cho phần RL.|pip trong môi trường Python phù hợp ROS.|Các package cần cài; bản thân file không phải dữ liệu train.
rl_training/README.md|Hướng dẫn RL hiện hành|Hướng dẫn chuẩn bị và chạy phần RL trên laptop.|Người dùng repo.|Lệnh train, resume, baseline/evaluate và hướng dẫn liên quan.
rl_training/README_WSL_LICH_SU.md|Hướng dẫn WSL lịch sử|Giữ nội dung hướng dẫn môi trường WSL cũ để tham khảo.|Người đọc tài liệu lịch sử.|Không mặc định mọi lệnh WSL phù hợp Ubuntu laptop hiện tại.
rl_training/ABLATION_SUITE.md|Hướng dẫn thực nghiệm đối chứng|Giải thích cách chạy các nhóm controller và phân tích log.|Người chạy PI/PID/PI + SAC.|Tài liệu cho runner và báo cáo; không phải chương trình thực thi.
rl_training/config/sac.yaml|Cấu hình SAC|Chứa ba nhóm environment, reward, sac: goal/hành lang/thời gian, trọng số reward và siêu tham số học.|train.py, môi trường ROS và công cụ thực nghiệm.|Cấu hình đầu vào; không chứa trọng số mạng đã học.
rl_training/config/pid.yaml|Profile PID vận hành|Lưu Kp, Ki, Kd và thời hằng lọc D được chọn.|Runner/controller khi chọn PID.|Profile riêng; PI và PI nền của SAC không dùng gain PID này.
rl_training/config/pid_gains.template.yaml|Mẫu gain PID|Khuôn điền tham số ứng viên; Kd/tau có thể null để buộc người dùng điền.|Người tạo profile thử.|Không dùng template chưa điền như bộ PID đã hiệu chỉnh.
rl_training/config/PID_CALIBRATION.json|Hồ sơ hiệu chỉnh PID|Lưu protocol/nguồn đo, ứng viên, lý do và bằng chứng xác minh của lần chọn profile.|Kết quả quy trình hiệu chỉnh.|Hồ sơ lịch sử có cấu trúc; không phải file nạp trọng số SAC.
rl_training/export_file_catalog.py|Xuất danh mục file|Quét toàn bộ tên file vật lý, đọc cấu trúc code dự án và tạo bảng HTML offline này.|--root và --output tùy chọn.|DANH_MUC_FILE_REPO.html; không import code robot, không chạy ROS/train, không đi theo liên kết thư mục.
src/nino_control/nino_control/effort_drive.py|Điều khiển truyền động bằng mô-men|Đổi cmd_vel thành mục tiêu bánh, đóng vòng PI/PID, quản lý residual, watchdog/freshness và limiter.|cmd_vel, JointState, residual và tham số controller.|Lệnh mô-men bánh, odometry và chẩn đoán; lệnh mô-men không tự chứng minh mô-men đo thực.
src/nino_control/nino_control/kinematics.py|Động học|Tính mục tiêu tốc độ hai bánh, tích phân odometry và các giới hạn lệnh.|Vận tốc thân/bánh, kích thước hình học và dt.|Giá trị số thuần Python, độc lập ROS. Odometry bánh là ước lượng và có thể ảnh hưởng bởi trượt.
src/nino_control/nino_control/derivative.py|Khâu đạo hàm|Tính D trên vận tốc đo, dùng dt và alpha=tau/(tau+dt), quản lý mẫu cũ/reset.|omega đo, timestamp, Kd và tau_D.|Thành phần D đã lọc; tránh derivative kick do bước nhảy setpoint.
src/nino_control/nino_control/residual.py|Cổng mô-men bổ sung|Giới hạn, kiểm tra thời hạn và quản lý residual torque độc lập ROS.|Action/mô-men residual và thời gian.|Residual được phép áp dụng hoặc về 0; bộ nền vẫn có vai trò điều khiển.
src/nino_control/nino_control/sensor_monitor.py|Theo dõi cảm biến|In thông tin gọn của IMU, encoder bánh và lidar đang nhận.|Các topic cảm biến.|Dòng trạng thái terminal; không phải bộ điều khiển.
src/nino_control/config/effort_drive.yaml|Cấu hình điều khiển mô-men|Thông số hình học, gain nền, tốc độ, mô-men, slew, watchdog và feedback.|Node effort_drive.|Tham số vận hành; thay file có thể đổi động học mà SAC từng học.
src/nino_control/config/controllers.yaml|Cấu hình controller ROS 2|Khai báo controller_manager, joint state broadcaster và controller mô-men.|ros2_control/Gazebo.|Các controller phụ trách nhận lệnh và xuất trạng thái khớp.
src/nino_navigation/scripts/episode_manager.py|Quản lý lượt chạy|Quản lý một đường tham chiếu và một FollowPath, state machine nhiệm vụ, theo dõi/huỷ và velocity gate.|Goal, Nav2 action, TF/odom và cấu hình episode.|Trạng thái episode và đường tham chiếu; thời gian tường giám sát cả trường hợp clock mô phỏng dừng.
src/nino_navigation/scripts/odom_warmup.py|Chờ odometry sẵn sàng|Đợi một luồng odometry liên tục có timestamp tiến lên trước khi chạy nhiệm vụ.|Topic odometry.|Điều kiện warmup đạt hoặc lỗi chờ; không tự sửa odometry.
src/nino_navigation/scripts/path_deviation.py|Độ lệch khỏi đường|Tính khoảng cách ngang từ điểm đến polyline tham chiếu bất biến.|Pose XY và các đoạn đường.|Sai số hình học; không tự lập đường hoặc lấy pose từ cảm biến.
src/nino_navigation/launch/navigation.launch.py|Khởi chạy điều hướng|Ghép các node/cấu hình cho phiên điều hướng Nav2.|Bản đồ và launch arguments.|Các tiến trình/node điều hướng theo launch; đọc phần CLI cấu trúc để xem tham số thực có trong file.
src/nino_navigation/launch/fixed_path.launch.py|Khởi chạy đường cố định|Tổ chức chạy nhiệm vụ theo đường tham chiếu cố định.|Reference, map và config.|Luồng episode theo đường đã chọn; dùng để kiểm soát hình học khi đối sánh.
src/nino_navigation/launch/mapping.launch.py|Khởi chạy tạo bản đồ|Tổ chức phiên SLAM để xây bản đồ.|Cảm biến/TF và cấu hình SLAM.|Các node phục vụ mapping; khác với định vị trên bản đồ có sẵn.
src/nino_navigation/config/nav2.yaml|Cấu hình Nav2|Tham số các thành phần định vị/điều hướng, costmap, planner/controller và kiểm tra đích.|Nav2 launch.|Các tham số, không phải đường tham chiếu đã lưu và không phải model SAC.
src/nino_navigation/config/slam.yaml|Cấu hình SLAM|Tham số xây bản đồ bằng slam_toolbox.|Mapping launch, scan và TF.|Cấu hình thuật toán tạo bản đồ.
src/nino_navigation/config/episode.yaml|Cấu hình lượt chạy|Tham số giám sát và tổ chức một nhiệm vụ.|Episode manager.|Điều kiện và thời gian cho episode theo code đang dùng.
src/nino_navigation/rviz/navigation.rviz|Hiển thị điều hướng|Bố cục RViz cho map, robot, TF, đường và cảm biến.|RViz và topic ROS.|Cấu hình hiển thị; không tạo chuyển động vật lý.
src/nino_description/launch/sim.launch.py|Khởi chạy mô phỏng|Tổ chức Gazebo, robot và các node mô phỏng liên quan.|World, robot description và launch arguments.|Phiên mô phỏng; không phải lệnh khởi động xe thật.
src/nino_description/urdf/nino.urdf.xacro|Mô tả robot Nino|Khai báo links/joints, hình học, quán tính, bánh/cảm biến và thành phần mô phỏng.|Xacro và các tham số mô tả.|Robot description sau khai triển; không phải dữ liệu học mạng.
src/nino_description/worlds/flat_world.sdf|Môi trường phẳng|Khai báo môi trường Gazebo nền phẳng.|Gazebo Sim.|Hình học/vật lý môi trường mô phỏng.
src/nino_description/worlds/long_hall.sdf|Hành lang dài|Khai báo môi trường hành lang và các thành phần vật lý/chướng ngại trong world.|Gazebo Sim.|World mô phỏng; không tự chứng minh tọa độ world trùng tọa độ map.
src/nino_description/config/bridge.yaml|Cầu nối ROS–Gazebo|Ánh xạ topic và kiểu message giữa Gazebo Transport và ROS.|ros_gz_bridge.|Luồng cảm biến/clock theo cấu hình; pipeline hiện hành không cần bridge GT.
src/nino_description/rviz/sensors.rviz|Hiển thị cảm biến|Bố cục RViz cho robot, TF, lidar và các tín hiệu được cấu hình.|RViz và topic ROS.|Chỉ cấu hình hiển thị; RViz khác với Gazebo mô phỏng vật lý.
src/nino_description/scripts/gz_sim_clean|Chạy Gazebo trong môi trường đã xử lý|Wrapper thiết lập môi trường trước khi gọi Gazebo; xem nội dung shell để biết biến được xử lý.|Launch mô phỏng và môi trường tiến trình.|Tiến trình Gazebo; không suy từ chữ clean rằng script xóa dữ liệu.
maps/hall_01.yaml|Thông tin bản đồ hành lang 01|Khai báo ảnh occupancy map, resolution, origin và ngưỡng ô.|Nav2/map_server.|Ghép với hall_01.pgm; không chứa trọng số học.
maps/hall_01.pgm|Ảnh bản đồ hành lang 01|Ảnh mức xám mã hóa ô trống/chiếm dụng/chưa biết theo ngưỡng YAML.|Map server.|Bản đồ 2D; không phải mô hình va chạm SDF.
"""
SPECS = {parts[0]: dict(zip(('meaning', 'role', 'inputs', 'outputs'), parts[1:]))
         for line in SPEC.strip().splitlines() if (parts := line.split('|'))}

TEST_ROLES = {
    'test_ablation_suite.py': 'Kiểm tra kế hoạch runner, đếm step thật từ checkpoint, cấu hình PID tách SAC, đường lưu sẵn, chạy tuần tự/resume và không ghi đè kết quả.',
    'test_amcl_pipeline.py': 'Kiểm tra thứ tự/đơn vị/scale observation giữ nguyên, observe/logger/terminal chỉ cần AMCL và cảm biến, không còn plugin/bridge/CLI GT chuyên dụng.',
    'test_calibrate_pid_measured.py': 'Kiểm tra chọn PID không nhận lượt AMCL thất bại chỉ vì Nav2 thành công, giữ lỗi dữ liệu, tái xử lý log cũ và giới hạn hai episode xác minh.',
    'test_dashboard.py': 'Kiểm tra đọc log tăng dần, dòng UTF-8/JSON dở, tính step resume, không đếm episode hai lần và chặn truy cập thư mục ngoài phạm vi.',
    'test_demo_server.py': 'Kiểm tra chế độ PI không gọi mạng SAC, SAC dùng deterministic action, đổi mode cần episode mới và giới hạn lệnh web hợp lệ.',
    'test_episode_outcome.py': 'Kiểm tra lỗi odom/hạ tầng không bị tính thành reward thất bại giả, lệch hành lang vẫn là task failure và điều kiện Nav2 success đúng.',
    'test_experiment_guard.py': 'Kiểm tra ngưỡng tài nguyên/nhiệt độ/thời hạn, phát hiện job khác, loại tiến trình cha và tuân thủ điều kiện chọn định trước.',
    'test_features.py': 'Kiểm tra dấu lỗi ngang/tiến độ/hướng, khử trọng lực khi đứng yên hoặc nghiêng và quan hệ reward thất bại.',
    'test_fixed3_metrics.py': 'Kiểm tra RMSE trọng số thời gian, lọc/jerk bằng đáp án biết trước, giữ lượt thất bại, dữ liệu thiếu/N/A, timestamp, mẫu trùng, residual gần 0 và phân biệt hai loại P95.',
    'test_gazebo_shutdown.py': 'Kiểm tra tiến trình Gazebo thuộc wrapper được dọn khi wrapper dừng, kể cả GUI server fork hoặc đổi session.',
    'test_localization_math.py': 'Kiểm tra nghịch đảo SE(2), cùng điểm vật lý, wrap góc, gap, mẫu trùng/cũ và không ngoại suy pose.',
    'test_long_run.py': 'Kiểm tra --steps là tổng đích, ghép đúng cặp model/replay, dùng snapshot lỗi hạ tầng, giữ kết quả cũ và lưu model/replay khi reset gặp lỗi.',
    'test_mission_measurement.py': 'Kiểm tra Nav2 outcome độc lập AMCL pass, ngưỡng bao gồm dấu bằng không làm tròn, thiếu pose không biến thành pass và giữ bằng chứng TF gốc.',
    'test_observation_refresh.py': 'Kiểm tra callback cảm biến cập nhật trước observation, không retry im lặng dữ liệu sai, callback ROS tiếp tục lúc learner bận và lỗi callback được truyền ra.',
    'test_runtime.py': 'Kiểm tra CPU/auto/CUDA không fallback GPU im lặng, toolkit Matplotlib tương thích và model/forward thật dùng thiết bị yêu cầu.',
    'test_source_pose_metrics.py': 'Kiểm tra trọng số theo timestamp nguồn, mẫu trùng/gap/thời gian lùi, wrap góc và pose cuối phải có bracket ở đúng ROS clock, không chọn pose thuận lợi nhất.',
    'test_startup.py': 'Kiểm tra retry trước goal có giới hạn, không retry lỗi runtime/cấu hình, nêu rõ node lifecycle chưa active, process exit, initialpose timeout và context phục hồi.',
    'verify_run.py': 'Đọc kết quả smoke đã lưu, kiểm tra gradient update/residual/giới hạn/reset, nạp model và gọi predict. Đây là xác minh model có thao tác thật, không chỉ test toán thuần.',
    'verify_startup_recovery.py': 'Script tích hợp chủ động mở SAC/Gazebo và chèn một lỗi trước goal để xác minh startup recovery. Không chạy cùng phiên mô phỏng khác; không được công cụ catalog thực thi.',
    'test_derivative.py': 'Kiểm tra D trên vận tốc đo với dt thay đổi, lọc khi phép đo không đổi, reset clock/mất mẫu và tham số lọc không hợp lệ.',
    'test_feedback_freshness_ros.py': 'Kiểm tra freshness chung cho PI/PID/residual, mẫu trùng/cũ/NaN/thời gian lùi/gap reset trạng thái và không tạo odometry giả.',
    'test_kinematics.py': 'Kiểm tra clamp, mục tiêu bánh khi đi thẳng/quay, tích phân odometry, chuẩn hóa yaw, giới hạn slew và soft speed cho phép hãm.',
    'test_pid_ros.py': 'Kiểm tra dấu D có tác dụng hãm, giới hạn mô-men giữ nguyên, Kd=0 giữ phép tính PI và mẫu lỗi/cũ reset I/D.',
    'test_residual.py': 'Kiểm tra residual mặc định tắt, biên độ/timeout, lệnh dừng, message sai, chuyển episode và giới hạn mô-men tổng.',
    'test_residual_ros.py': 'Kiểm tra adapter mô-men qua ROS topic thật với các trạng thái dừng/thất bại/timeout; không dùng động cơ hoặc vật lý Gazebo.',
    'test_episode_manager.py': 'Kiểm tra bằng ROS action server trong tiến trình: số yêu cầu lập đường, tái dùng reference, endpoint sai, lệch đường, huỷ và lỗi controller; không mô phỏng vật lý.',
    'test_odom_warmup.py': 'Kiểm tra một mẫu gần đây chưa đủ sẵn sàng; yêu cầu luồng liên tục có timestamp tiến lên, gap/reset clock khởi động lại warmup.',
    'test_path_deviation.py': 'Kiểm tra khoảng cách đến phần trong đoạn thẳng thay vì chỉ đỉnh gần nhất, endpoint/điểm trùng/góc rẽ/ngưỡng và input sai.',
    'verify_fixed_path.py': 'Script tích hợp mở/dừng mô phỏng riêng để xác minh đường cố định. Cần chạy đơn lẻ trong môi trường ROS; không được danh mục này thực thi.',
}

GROUPS = {
    'project': 'Code, cấu hình và tài liệu dự án',
    'test': 'Kiểm thử / xác minh dự án',
    'archive': 'Bản lưu lịch sử — không mặc định đang dùng',
    'result': 'Log, checkpoint và kết quả thực nghiệm',
    'dependency': 'Thư viện / môi trường Python bên thứ ba',
    'generated': 'Build, cài đặt và cache sinh tự động',
    'git': 'Dữ liệu quản lý phiên bản Git',
    'editor': 'Dữ liệu của IDE / môi trường phát triển',
    'catalog': 'Bảng danh mục đang xuất',
}


def group_for(path):
    parts = Path(path).parts
    if '.git' in parts:
        return 'git'
    if any(p.startswith('.venv') or p in ('site-packages', 'dist-packages') for p in parts):
        return 'dependency'
    if parts[0] == 'archive':
        return 'archive'
    if parts[0] in ('build', 'install', 'log') or any(p in ('__pycache__', '.pytest_cache') for p in parts):
        return 'generated'
    if parts[0] in ('benchmark_results', 'bags') or 'runs' in parts:
        return 'result'
    if parts[0] == '.vscode':
        return 'editor'
    if 'test' in parts or 'tests' in parts:
        return 'test'
    return 'project'


def python_structure(path):
    """Chỉ parse AST; không import module và không đọc model/replay bằng pickle."""
    if path.suffix != '.py' or path.stat().st_size > 1_000_000:
        return {}
    try:
        tree = ast.parse(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, SyntaxError) as exc:
        return {'parse_note': f'Không phân tích được AST: {type(exc).__name__}'}
    names = [n.name for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))]
    flags = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        called = node.func.attr if isinstance(node.func, ast.Attribute) else (node.func.id if isinstance(node.func, ast.Name) else '')
        if called not in ('add_argument', 'DeclareLaunchArgument'):
            continue
        values = [a.value for a in node.args if isinstance(a, ast.Constant) and isinstance(a.value, str)]
        if not values:
            continue
        item = {'name': ', '.join(values)}
        for kw in node.keywords:
            if kw.arg in ('required', 'default', 'choices', 'default_value'):
                try:
                    item[kw.arg] = ast.literal_eval(kw.value)
                except (ValueError, TypeError):
                    item[kw.arg] = ast.unparse(kw.value)
        flags.append(item)
    tests = [n.name for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
             and n.name.startswith('test_')]
    return dict(doc=(ast.get_docstring(tree) or '')[:2200], symbols=names, flags=flags, tests=tests)


ARTIFACTS = {
    'steps.jsonl': ('Dữ liệu từng bước RL', 'Mỗi dòng là một bản ghi JSON ở một bước tương tác: observation/đường đi/reward/action và trạng thái theo schema của phiên. Dùng để phân tích, không phải trọng số mạng.'),
    'telemetry.jsonl': ('Dữ liệu cảm biến và controller theo thời gian', 'Luồng ghi phục vụ metric/benchmark. Cần đọc schema và timestamp nguồn trước tính chỉ số; file lịch sử có thể khác schema hiện hành.'),
    'monitor.csv': ('Bảng Monitor của môi trường RL', 'Theo dõi reward/tổng thời gian hoặc độ dài episode do SB3 Monitor ghi; khác dữ liệu cảm biến tần số cao.'),
    'metadata.json': ('Thông tin mô tả lần chạy', 'Nguồn model, mode, seed, thứ tự observation/action và các bằng chứng cấu hình tùy phiên. Không suy số step chỉ từ tên thư mục.'),
    'config.yaml': ('Bản chụp cấu hình lần chạy', 'Cấu hình được lưu để tái lập phiên; không tự đồng bộ với file cấu hình vận hành sau khi đã lưu.'),
    'controller.yaml': ('Bản chụp tham số controller', 'Tham số controller của nhóm/lần thử, dùng đối chiếu gain và giới hạn. Không mặc định là profile vận hành hiện tại.'),
    'worker_spec.json': ('Đặc tả tiến trình chạy thử', 'Đầu vào có cấu trúc cho worker do runner tạo; mô tả phiên thực nghiệm cụ thể.'),
    'status.json': ('Trạng thái quy trình', 'Trạng thái job, tiến trình/pha và thời điểm cập nhật theo script tạo; không phải bằng chứng nhiệm vụ thành công nếu chưa có kết quả episode.'),
    'result.json': ('Kết quả lần chạy', 'Kết quả tóm tắt do chương trình tạo. Cần phân biệt số bước train, outcome Nav2 và điều kiện task pass.'),
    'manifest.json': ('Bản kê nguồn và đầu ra', 'Danh sách đường dẫn/hash/cấu hình của lần thực nghiệm hoặc bản sao tùy công cụ tạo. Dùng truy xuất nguồn gốc.'),
    'reference_path.json': ('Đường tham chiếu đã lưu', 'Hình học đường và frame để các lượt dùng cùng tham chiếu; không phải quỹ đạo robot thực sự đã đi.'),
    'startup_recovery.jsonl': ('Lịch sử phục hồi khởi động', 'Mỗi dòng ghi sự kiện lỗi/thử lại trước nhiệm vụ, giúp công khai lỗi hạ tầng.'),
    'device_verification.json': ('Bằng chứng thiết bị chạy', 'Kết quả kiểm tra model/forward trên CPU hoặc CUDA, tùy công cụ tạo.'),
    'resources.jsonl': ('Log tài nguyên', 'Các snapshot tài nguyên/thời gian và kiểm tra điều kiện dừng của guard.'),
    'episodes.csv': ('Chỉ số từng episode', 'Mỗi hàng mô tả một lượt thử hoặc nhóm dữ liệu episode; không loại lượt thất bại chỉ để cải thiện trung bình.'),
    'summary.csv': ('Bảng tổng hợp', 'Chỉ số tổng hợp các lượt; xem cách định nghĩa chỉ số và số mẫu trong báo cáo đi kèm.'),
    'summary.json': ('Tổng hợp dạng JSON', 'Các thống kê có cấu trúc để chương trình/báo cáo đọc lại; null có thể biểu thị dữ liệu không đủ.'),
    'comparison_table.tex': ('Bảng so sánh LaTeX', 'Mã bảng để đưa vào tài liệu khoa học; phải biên dịch LaTeX mới có hình bảng dàn trang.'),
    'report_PI_PID_SAC.html': ('Báo cáo ba controller', 'Trang báo cáo mở bằng trình duyệt. Kết quả lịch sử phản ánh phiên đã ghi, không tự chạy lại khi mở.'),
    'audit.json': ('Bằng chứng kiểm tra dữ liệu', 'Thông tin kiểm tra tính đầy đủ/tương thích hoặc nguồn phép đo theo công cụ xuất báo cáo.'),
    'report_data.json': ('Dữ liệu nguồn báo cáo', 'Dữ liệu đã tổng hợp dùng dựng bảng/biểu đồ và kiểm tra lại báo cáo.'),
    'protocol.json': ('Quy trình đã chốt', 'Ngân sách, ứng viên, điều kiện và tiêu chí định trước cho thực nghiệm/tuning.'),
    '.suite.lock': ('Khóa phiên chạy', 'File khóa phối hợp các job; chỉ thấy file tồn tại không đủ để kết luận khóa vẫn được giữ bởi tiến trình sống.'),
}


def fallback(path):
    p = Path(path)
    name, suffix = p.name, p.suffix.lower()
    if name in ARTIFACTS:
        return ARTIFACTS[name]
    if suffix == '.zip':
        return 'Archive ZIP / checkpoint tùy file', 'Trong runs, tên model hoặc sac_* thường chỉ checkpoint SB3; chỉ tên và đuôi không chứng minh thuật toán/số step. Catalog không giải nén hay nạp model.'
    if suffix == '.pkl':
        return 'Dữ liệu Python tuần tự hóa', 'Trong runs thường lưu replay buffer cho resume; không nạp pickle trong công cụ danh mục. File này không thay thế model.'
    if suffix in ('.pyc', '.pyo'):
        return 'Cache bytecode Python', 'Sinh khi import/chạy Python để tái sử dụng bytecode; mã nguồn tương ứng nằm ở file .py. Không chỉnh trực tiếp.'
    if name == '__init__.py':
        return 'Khởi tạo package Python', 'Đánh dấu/khởi tạo package và có thể khai báo export. File ngắn hoặc trống không đồng nghĩa thiếu chức năng.'
    if name == 'package.xml':
        return 'Khai báo package ROS', 'Tên, phiên bản, maintainer, loại build và dependency; ROS/rosdep/colcon dùng để nhận diện package.'
    if name == 'CMakeLists.txt':
        return 'Quy tắc build/cài đặt CMake', 'Khai báo project và các thành phần cần build/cài đặt; trong package mô tả/điều hướng thường cài launch/config/scripts.'
    if name in ('setup.py', 'setup.cfg'):
        return 'Đóng gói/cài đặt Python', 'Khai báo package, data_files hoặc console entry points và cấu hình đường dẫn cài. Đọc code node để xem hành vi robot.'
    if name == '.gitkeep':
        return 'Giữ thư mục trong Git', 'File đánh dấu để Git có thể lưu một thư mục chưa có nội dung khác; không được robot sử dụng như dữ liệu.'
    if name == 'Dockerfile':
        return 'Công thức tạo image Docker', 'Các bước tạo môi trường container; không phải lệnh trực tiếp chạy robot.'
    if p.parent.name == 'resource':
        return 'Dấu nhận diện package', 'File marker cho chỉ mục tài nguyên ament giúp ROS nhận diện package đã cài.'
    types = {
        '.py': ('Mã nguồn Python', 'Chức năng cụ thể cần đọc docstring/cấu trúc bên dưới; không xác nhận hành vi chỉ từ tên file.'),
        '.yaml': ('Cấu hình YAML', 'Lưu tham số/bản kê cho chương trình khác đọc. Cùng tên config ở runs là snapshot, không tự thay cấu hình hiện hành.'),
        '.yml': ('Cấu hình YAML', 'Dữ liệu cấu hình có cấu trúc; cần chương trình đọc để có tác dụng.'),
        '.json': ('Dữ liệu JSON', 'Thông tin có cấu trúc; công cụ danh mục không tự suy rằng file chứa kết quả đạt yêu cầu.'),
        '.jsonl': ('Dữ liệu JSON theo dòng', 'Mỗi dòng là một bản ghi; schema tùy script tạo và có thể thay đổi giữa các phiên lịch sử.'),
        '.csv': ('Dữ liệu dạng bảng', 'Mở bằng LibreOffice/Excel hoặc đọc bằng Python; tên cột, đơn vị và timestamp phải đối chiếu trước khi tính chỉ số.'),
        '.md': ('Tài liệu Markdown', 'Hướng dẫn, ghi chú hoặc bằng chứng kiểm chứng. Bản lưu lịch sử có thể không phản ánh code hiện hành.'),
        '.txt': ('Văn bản thuần', 'Nội dung chữ hoặc đầu ra công cụ; vai trò cụ thể tùy đường dẫn, không phải mọi file .txt đều là hướng dẫn.'),
        '.log': ('Nhật ký chương trình', 'Lưu thông báo stdout/stderr để chẩn đoán. navigation.log thuộc Nav2, simulation.log thuộc mô phỏng; log khác theo tiến trình tạo.'),
        '.html': ('Trang/báo cáo HTML', 'Mở bằng trình duyệt để xem giao diện; một trang live có thể cần máy chủ API, còn báo cáo nhúng dữ liệu thường xem offline.'),
        '.tex': ('Mã LaTeX', 'Nguồn văn bản/bảng cần biên dịch bằng LaTeX để có tài liệu trình bày.'),
        '.svg': ('Hình vector SVG', 'Biểu đồ/hình dạng mô tả bằng XML; mở bằng trình duyệt hoặc phần mềm vẽ vector.'),
        '.png': ('Ảnh PNG', 'Hình ảnh/biểu đồ đã xuất; ảnh không tự cập nhật khi có log mới.'),
        '.jpg': ('Ảnh JPEG', 'Ảnh bitmap; không phải mã nguồn.'),
        '.jpeg': ('Ảnh JPEG', 'Ảnh bitmap; không phải mã nguồn.'),
        '.pgm': ('Ảnh mức xám', 'Trong maps thường là occupancy map, đi kèm YAML quy định resolution/origin/ngưỡng.'),
        '.stl': ('Lưới hình học 3D STL', 'Mô tả bề mặt chi tiết robot; base_link là thân, wheel là bánh, caster là bánh tự lựa, bracket là giá đỡ. URDF tham chiếu mesh để hiển thị/va chạm theo cấu hình; không chứa bộ điều khiển.'),
        '.dae': ('Mô hình 3D Collada', 'Hình học/vật liệu mô hình 3D để công cụ hiển thị hoặc mô phỏng nạp; không phải code điều khiển.'),
        '.sdf': ('Mô tả mô phỏng SDF', 'Khai báo world hoặc model cho Gazebo, gồm hình học/vật lý/plugin; không phải bản đồ Nav2.'),
        '.xacro': ('Mẫu XML robot', 'Khai triển thành URDF/XML bằng xacro; mô tả links/joints/hình học và các macro.'),
        '.rviz': ('Cấu hình hiển thị RViz', 'Lưu displays, topic, frame và góc nhìn; không tự tạo dữ liệu cảm biến.'),
        '.xml': ('Dữ liệu/cấu hình XML', 'Cấu trúc đánh dấu dùng cho package, plugin, DDS hoặc mô tả khác; vai trò phụ thuộc đường dẫn.'),
        '.sh': ('Script shell', 'Các lệnh hệ thống được thực thi khi chạy script; không chạy thử tự động từ danh mục.'),
        '.bash': ('Thiết lập/lệnh Bash', 'Trong install thường thiết lập môi trường để ROS tìm package; cần source trong terminal khi dùng.'),
        '.dsv': ('Mô tả biến môi trường', 'Colcon/ament sinh để ghép đường dẫn package vào môi trường khi source setup.'),
        '.so': ('Thư viện nhị phân chia sẻ', 'Mã đã biên dịch được chương trình/extension nạp; phụ thuộc kiến trúc và phiên bản môi trường.'),
        '.db3': ('Cơ sở dữ liệu SQLite', 'Trong bags thường lưu message rosbag2; cần metadata và công cụ ROS để đọc/phát lại đúng.'),
        '.mcap': ('Bản ghi MCAP', 'Dữ liệu message theo thời gian, thường dùng rosbag2; không phải model SAC.'),
        '.patch': ('Bản vá thay đổi', 'Lưu khác biệt giữa các phiên bản; đọc để biết thay đổi, không áp dụng tự động khi mở.'),
        '.o': ('Object đã biên dịch', 'File trung gian build; không sửa để thay code nguồn.'),
        '.cmake': ('Module CMake', 'Quy tắc build hoặc kết quả cấu hình tự sinh; phân biệt theo thư mục nguồn/build.'),
    }
    return types.get(suffix, ('File chưa xác định riêng', 'Liệt kê đúng đường dẫn và loại lưu trữ; chưa đủ bằng chứng để giải thích chức năng chuyên biệt. Không giả định file này tham gia pipeline.'))


def describe(rel, path, group, kind):
    meaning, role = fallback(rel)
    item = dict(meaning=meaning, role=role, inputs='', outputs='', evidence='Theo loại file và vị trí; không thực thi nội dung.')
    if group == 'dependency':
        item.update(role='File thuộc môi trường Python/thư viện bên thứ ba; không phải code robot do dự án tự viết. ' + role,
                    outputs='Giữ theo quản lý của môi trường pip/venv; không chỉnh lẻ file để thay controller.')
    elif group == 'git':
        item.update(meaning='Dữ liệu nội bộ Git', role='Git dùng đường dẫn này để quản lý lịch sử, object, ref, index hoặc cấu hình. Không phải log cảm biến hay mã train.',
                    outputs='Không chỉnh hoặc xóa thủ công khi đang làm việc với repo.')
    elif group == 'editor':
        item.update(role='Cấu hình hoặc chỉ mục của VS Code/extension. ' + role,
                    outputs='Phục vụ IDE; không phải trọng số mạng hoặc phần điều khiển xe.')
    elif group == 'generated':
        item['role'] = 'Bản sinh tự động/cache hoặc bản cài từ mã nguồn; vị trí build/install/log quyết định vai trò. ' + role
    elif rel in SPECS:
        item.update(SPECS[rel], evidence='Mô tả theo code/tài liệu hiện hành; AST nếu là Python. Không chứng minh hiệu năng phần cứng.')
    if group == 'archive':
        item['role'] = 'BẢN LỊCH SỬ, không mặc định đang được gọi. ' + role
        item['outputs'] = 'Không gán hành vi phiên bản hiện tại cho bản lưu; xem docstring/các hàm/CLI trích trực tiếp bên dưới.'
    if kind == 'symlink_dir':
        item.update(meaning='Liên kết tới thư mục', role='Đường dẫn liên kết thư mục, không phải một bản sao riêng. Catalog liệt kê liên kết nhưng không quét tiếp đích để tránh lặp hoặc đọc ra ngoài workspace.')
    elif kind == 'symlink':
        item['outputs'] += ' Đây là symlink, không tính kích thước file đích thành dữ liệu mới.'
    elif kind != 'file':
        item['role'] += ' Đây không phải file thường; công cụ không đọc nội dung.'
    if group in ('project', 'test', 'archive') and kind == 'file':
        info = python_structure(path)
        item.update(info)
        if group == 'test' and path.suffix == '.py':
            target = path.stem.removeprefix('test_').removeprefix('verify_')
            related = next((v for k, v in SPECS.items() if Path(k).stem == target), None)
            item['meaning'] = ('Kiểm thử: ' if path.name.startswith('test_') else 'Xác minh: ') + (related['meaning'] if related else target.replace('_', ' '))
            item['role'] = 'Kiểm tra các điều kiện được thể hiện bằng những test/hàm trích trực tiếp bên dưới. ' + (('Thành phần liên quan: '+related['role']) if related else '')
            if path.name in TEST_ROLES:
                item['role'] = TEST_ROLES[path.name]
            item['outputs'] = 'Định nghĩa hoặc script kiểm tra; có file xác minh tích hợp mở ROS/Gazebo. Danh mục chỉ parse AST, KHÔNG chạy test và không tuyên bố test đạt.'
    if 'idual SAC training' in rel:
        item.update(meaning='File ghi đầu ra Git có tên bất thường', role='Nội dung đã quan sát là thống kê thay đổi dạng git diff --stat có mã màu terminal; không phải script train SAC.', outputs='Giữ nguyên file; tên không được runner dùng làm lệnh train.')
    return item


def inventory(root, output):
    rows, descriptions, ids, errors = [], [], {}, []
    def add(path, linked_dir=False):
        rel = path.relative_to(root).as_posix()
        try:
            s = path.lstat()
            kind = ('symlink_dir' if linked_dir else 'symlink') if stat.S_ISLNK(s.st_mode) else ('file' if stat.S_ISREG(s.st_mode) else 'special')
            group = 'catalog' if path == output else group_for(rel)
            desc = describe(rel, path, group, kind)
            extra = {}
            if kind.startswith('symlink'):
                extra = dict(link=os.readlink(path), broken=not path.exists())
            if path == output:
                desc = dict(meaning='Danh mục HTML này', role='Ảnh chụp danh sách file tại thời điểm xuất; thay đổi repo sau đó cần xuất lại.', inputs='Toàn bộ đường dẫn dưới root.', outputs='Trang offline, tìm kiếm/lọc/phân trang; không quét theo symlink thư mục.', evidence='File đầu ra tự liệt kê; kích thước để trống để tránh phụ thuộc vòng.')
            key = json.dumps(desc, ensure_ascii=False, sort_keys=True)
            if key not in ids:
                ids[key] = len(descriptions); descriptions.append(desc)
            rows.append([rel, group, ids[key], None if path == output else s.st_size,
                         datetime.fromtimestamp(s.st_mtime, ZoneInfo('Asia/Bangkok')).isoformat(timespec='seconds'), kind, extra])
        except OSError as exc:
            errors.append(f'{rel}: {exc}')
    for base, dirs, files in os.walk(root, followlinks=False, onerror=lambda e: errors.append(str(e))):
        dirs.sort(); files.sort()
        for name in dirs[:]:
            p = Path(base)/name
            if p.is_symlink():
                add(p, linked_dir=True); dirs.remove(name)
        for name in files:
            add(Path(base)/name)
    rows.sort(key=lambda row: (row[1] not in ('project', 'test', 'catalog'), row[0]))
    return rows, descriptions, errors


PAGE = r'''<!doctype html>
<html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Danh mục toàn bộ file — Nino Robotics</title>
<style>
:root{font-family:system-ui,-apple-system,Segoe UI,sans-serif;color:#203046;background:#eef2f6;font-size:15px}*{box-sizing:border-box}body{margin:0}header{background:#10283f;color:#fff;padding:34px max(24px,4vw)}h1{font-size:clamp(26px,3vw,40px);margin:8px 0 12px;line-height:1.15}header p{max-width:1000px;color:#d2e2f0;line-height:1.7;margin:7px 0}.eyebrow{color:#7ee0c7;font-size:13px;font-weight:700;letter-spacing:.12em}main{padding:24px max(20px,4vw) 50px;max-width:1800px;margin:auto}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-bottom:18px}.card{background:white;padding:17px 20px;border:1px solid #dbe4ec;border-radius:10px}.card b{display:block;font-size:29px;color:#125e7d}.card span{font-size:13px;color:#526579}section{background:#fff;border:1px solid #dbe4ec;border-radius:10px;padding:20px;margin:16px 0}h2{font-size:20px;margin:0 0 12px}h3{font-size:16px;margin:16px 0 8px}p,li{line-height:1.7}code{font-family:ui-monospace,Consolas,monospace;overflow-wrap:anywhere;font-size:.91em}pre{padding:12px;background:#f0f5f8;border-radius:6px;white-space:pre-wrap}a{color:#076a8b}button,select,input{font:inherit;border:1px solid #b6c6d3;border-radius:6px;padding:10px;background:white;color:#203046}button{cursor:pointer}button:hover{background:#e6f4f7}button.primary{background:#116a85;border-color:#116a85;color:#fff}button:disabled{opacity:.45;cursor:default}.filters{display:grid;grid-template-columns:minmax(260px,2fr) 1fr 1fr auto;gap:10px}.filters label{display:flex;flex-direction:column;gap:6px;font-size:13px;font-weight:600}input{width:100%}.help{font-size:13px;color:#53677a}.toolbar{display:flex;justify-content:space-between;align-items:center;gap:12px;margin:15px 0;flex-wrap:wrap}.pager{display:flex;align-items:center;gap:8px}.tablewrap{overflow:auto;border:1px solid #dbe4ec;border-radius:8px;max-height:76vh}table{width:100%;border-collapse:collapse;min-width:960px;background:white}th{position:sticky;top:0;background:#e6eef4;z-index:1;text-align:left;padding:12px;font-size:13px}td{border-top:1px solid #e5ecf1;padding:13px;vertical-align:top;line-height:1.6}tr:nth-child(even){background:#f9fbfd}td.path{width:29%}td.role{width:33%}td.use{width:26%}.badge{display:inline-block;background:#e4eef6;color:#315370;padding:2px 7px;border-radius:4px;font-size:11px;margin:6px 4px 3px 0}.meaning{font-weight:700;margin-bottom:5px}.muted{color:#607487;font-size:12px}.detail{margin-top:8px}summary{cursor:pointer;color:#076a8b;font-weight:600;font-size:13px}.detail p{font-size:13px}.flaglist{margin:7px 0;padding-left:20px;font-size:12px}.notice{border-left:4px solid #d3a13a;background:#fff9e9;padding:10px 14px;font-size:13px}.flow{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.flow div{background:#f0f6f9;padding:14px;border-radius:7px}.flow strong{display:block;margin-bottom:8px}.foot{font-size:12px;color:#65798b;overflow-wrap:anywhere}.empty{padding:30px;text-align:center;color:#5a6f83}noscript{display:block;background:#fff3cd;padding:20px}.shortcut{margin-right:6px;margin-top:6px;font-size:13px}.glossary{columns:3;line-height:1.8;font-size:13px}.smalltable{min-width:0}.smalltable th{position:static}.scope-note{padding:10px 0;color:#3c566b;font-size:13px}@media(max-width:900px){.cards{grid-template-columns:repeat(2,1fr)}.filters{grid-template-columns:1fr 1fr}.flow{grid-template-columns:1fr}.glossary{columns:1}}@media print{header{background:white;color:black}header p{color:black}.filters,.pager,.shortcut{display:none}.tablewrap{max-height:none;overflow:visible}th{position:static}.cards{grid-template-columns:repeat(4,1fr)}.notice:after{content:' Khi in, chỉ các hàng của trang hiện tại được in. Dùng xuất CSV để lấy mọi hàng đang lọc.'}}
</style></head><body>
<header><div class="eyebrow">NINO ROBOTICS / BẢN ĐỒ MÃ NGUỒN</div><h1>Từng file làm nhiệm vụ gì?</h1><p>Danh mục đầy đủ đường dẫn trong workspace, kèm giải thích tiếng Việt. Bao gồm mã nguồn, cấu hình, test, bản lưu, kết quả và thư viện phụ thuộc. Không rút gọn các bản sao thành một hàng.</p><p id="stamp"></p></header>
<main><noscript>Hãy bật JavaScript để tìm kiếm và phân trang. Dữ liệu đã nhúng trong HTML; không cần mạng Internet.</noscript>
<div class="cards"><div class="card"><b id="total">—</b><span>File và liên kết được kiểm kê</span></div><div class="card"><b id="own">—</b><span>Code, cấu hình, tài liệu và test dự án</span></div><div class="card"><b id="deps">—</b><span>File thư viện / môi trường Python</span></div><div class="card"><b id="others">—</b><span>Log, archive, cache và file khác</span></div></div>
<section><h2>Đọc đúng phạm vi danh mục</h2><p>Đây là ảnh chụp file tại thời điểm xuất. Mặc định bảng ưu tiên file dự án; chọn <b>Tất cả file</b> để xem cả thư viện, lịch sử và file sinh tự động. Mỗi đường dẫn có một hàng riêng, kể cả file trùng tên ở nhiều thư mục.</p><div class="notice" id="coverage"></div><p class="help">Mô tả chuyên biệt áp dụng cho code hiện hành. Bản archive dùng thông tin trích từ chính bản lưu; file phụ thuộc/nhị phân được giải thích theo loại và vị trí. Công cụ không import code robot, không nạp pickle/model, không mở Gazebo và không chạy test robot. Không suy chức năng chuyên biệt khi chưa có bằng chứng.</p><details><summary>Ý nghĩa đuôi file và thuật ngữ</summary><div class="glossary">.py: code Python<br>.yaml: tham số cấu hình<br>.json: dữ liệu có cấu trúc<br>.jsonl: một bản ghi JSON mỗi dòng<br>.csv: bảng dữ liệu<br>.md: tài liệu Markdown<br>.html: trang web/báo cáo<br>.tex: nguồn LaTeX<br>.sdf: world/model Gazebo<br>.urdf.xacro: mô tả robot dạng macro<br>.rviz: cấu hình hiển thị RViz<br>.pgm: ảnh bản đồ mức xám<br>.zip: archive; có thể là model SB3<br>.pkl: dữ liệu Python tuần tự hóa<br>run: lần chạy; episode: một lượt nhiệm vụ<br>step: một bước tương tác RL<br>checkpoint: trạng thái model đã lưu<br>residual: phần điều khiển bổ sung<br>ablation: thêm/bớt thành phần để đối chứng<br>benchmark: đánh giá theo bộ điều kiện/chỉ số<br>pose: vị trí và hướng; odometry: ước lượng chuyển động<br>timestamp: thời điểm nguồn dữ liệu<br>gain: hệ số điều khiển; slew: tốc độ thay đổi lệnh</div></details></section>
<section><h2>Ba luồng dễ nhầm trong repo</h2><div class="flow"><div><strong>1. Train / resume SAC</strong><code>train.py</code><p>Trực tiếp tạo môi trường, học hoặc evaluate. Có <code>--gui</code>.</p><code>long_run.py → train.py</code><p>Wrapper cho một job resume dài. Lưu <code>status.json</code> và <code>train.log</code>; không tự so sánh ba controller.</p></div><div><strong>2. Chạy đối chứng</strong><code>run_ablation_suite.py<br>↓ rl_training/ablation_suite.py<br>↓ ros_env.py + Nav2 + controller</code><p>Cho từng controller chạy và lưu log. Đây là bước tạo dữ liệu thực nghiệm.</p></div><div><strong>3. Phân tích sau khi chạy</strong><code>benchmark_pi_pid_sac.py</code><p>Đọc ba thư mục log, xuất bảng và biểu đồ offline. Không train và không tự mở Gazebo.</p></div></div><details open><summary>long_run.py — giải thích cụ thể</summary><div id="longrun"></div></details></section>
<section><h2>Tra cứu từng file</h2><div class="filters"><label>Tìm tên, đường dẫn hoặc công dụng<input id="query" type="search" placeholder="Ví dụ: long_run.py, reward, encoder, telemetry…"></label><label>Phạm vi<select id="scope"><option value="own">Code / cấu hình / tài liệu / test hiện hành</option><option value="all">Tất cả file trong workspace</option></select></label><label>Nhóm file<select id="group"><option value="all">Mọi nhóm trong phạm vi đã chọn</option></select></label><label>Số hàng / trang<select id="pageSize"><option>50</option><option selected>100</option><option>200</option></select></label></div><div><button class="shortcut" data-query="long_run.py">long_run.py</button><button class="shortcut" data-query="train.py">train.py</button><button class="shortcut" data-query="reward">Reward</button><button class="shortcut" data-query="effort_drive">Điều khiển bánh</button><button class="shortcut" id="showArchive">Archive</button><button class="shortcut" id="reset">Xóa bộ lọc</button></div><div class="scope-note" id="scopeNote"></div><div class="toolbar"><strong id="matches" aria-live="polite"></strong><div class="pager"><button id="export">Xuất CSV các hàng đang lọc</button><button id="prev">← Trước</button><span id="page"></span><button id="next">Sau →</button></div></div><div class="tablewrap"><table><thead><tr><th>File / đường dẫn / nhóm</th><th>Tên có nghĩa gì và file làm gì?</th><th>Đầu vào — đầu ra / cách dùng</th><th>Chi tiết</th></tr></thead><tbody id="rows"></tbody></table></div></section>
<section><h2>Thống kê phạm vi và lỗi kiểm kê</h2><table class="smalltable"><thead><tr><th>Nhóm</th><th>Số đường dẫn</th></tr></thead><tbody id="groups"></tbody></table><p id="errors" class="help"></p><p class="foot" id="root"></p><p class="foot">Nút “Mở file” chỉ trỏ đến file cục bộ tương đối với vị trí báo cáo. Di chuyển riêng HTML vẫn xem được bảng, nhưng liên kết đến mã nguồn cần giữ cùng cây thư mục. Không hiển thị nội dung mật khẩu/token trong bảng.</p></section></main>
<script id="catalog-data" type="application/json">__DATA__</script>
<script>
'use strict';
const data=JSON.parse(document.getElementById('catalog-data').textContent),all=data.rows,defs=data.descriptions;
const $=id=>document.getElementById(id),format=n=>n.toLocaleString('vi-VN');
const normalize=s=>String(s||'').normalize('NFD').replace(/[\u0300-\u036f]/g,'').replace(/đ/g,'d').replace(/Đ/g,'D').toLowerCase();
const labels=data.groups;
const search=all.map(r=>{const d=defs[r[2]];return normalize([r[0],labels[r[1]],d.meaning,d.role,d.inputs,d.outputs,(d.symbols||[]).join(' ')].join(' '))});
let filtered=[],page=0;
function node(tag,text,cls){let n=document.createElement(tag);if(text!==undefined)n.textContent=text;if(cls)n.className=cls;return n;}
function bytes(n){if(n===null)return 'Đang xuất';if(n<1024)return n+' B';if(n<1048576)return (n/1024).toFixed(1)+' KiB';return (n/1048576).toFixed(1)+' MiB';}
function detail(d){const wrap=node('details',undefined,'detail');wrap.append(node('summary','Cấu trúc & bằng chứng'));wrap.append(node('p',d.evidence||'Phân loại theo đường dẫn.'));if(d.doc){wrap.append(node('b','Docstring nguyên văn trong file:'));wrap.append(node('pre',d.doc));}if(d.symbols?.length){wrap.append(node('b','Hàm / lớp cấp module:'));wrap.append(node('pre',d.symbols.join(', ')));}if(d.flags?.length){wrap.append(node('b','Tham số trích từ AST (không thực thi):'));let ul=node('ul',undefined,'flaglist');for(const f of d.flags){let text=f.name;if(f.required===true)text+=' — bắt buộc';if(Object.hasOwn(f,'default'))text+='; mặc định: '+JSON.stringify(f.default);if(Object.hasOwn(f,'default_value'))text+='; mặc định launch: '+JSON.stringify(f.default_value);if(f.choices)text+='; lựa chọn: '+JSON.stringify(f.choices);ul.append(node('li',text));}wrap.append(ul);}if(d.tests?.length){wrap.append(node('b','Test được định nghĩa (chưa chạy):'));wrap.append(node('pre',d.tests.join('\n')));}if(d.parse_note)wrap.append(node('p',d.parse_note));return wrap;}
function render(){const size=Number($('pageSize').value),pages=Math.max(1,Math.ceil(filtered.length/size));page=Math.min(page,pages-1);$('rows').replaceChildren();for(const index of filtered.slice(page*size,(page+1)*size)){const r=all[index],d=defs[r[2]],tr=node('tr');tr.dataset.path=r[0];const a=node('td',undefined,'path');a.append(node('code',r[0]),node('br'),node('span',labels[r[1]],'badge'));if(r[5]!=='file')a.append(node('span',r[5],'badge'));const link=node('a','Mở file');link.href=data.prefix+r[0].split('/').map(encodeURIComponent).join('/');link.target='_blank';link.rel='noopener';if(!r[6]?.broken)a.append(node('br'),link);else a.append(node('p','Liên kết đích không còn tồn tại','muted'));if(r[6]?.link)a.append(node('p','Đích liên kết: '+r[6].link,'muted'));const b=node('td',undefined,'role');b.append(node('div',d.meaning,'meaning'),node('div',d.role));const c=node('td',undefined,'use');if(d.inputs)c.append(node('b','Đầu vào: '),node('span',d.inputs),node('br'));if(d.outputs)c.append(node('b','Đầu ra / lưu ý: '),node('span',d.outputs));if(!d.inputs&&!d.outputs)c.append(node('span','Xem mô tả vai trò và đường dẫn; không suy thêm hành vi khi chưa có bằng chứng.'));const e=node('td');e.append(node('div',bytes(r[3]),'muted'),node('div',r[4],'muted'),detail(d));tr.append(a,b,c,e);$('rows').append(tr);}if(!filtered.length){const tr=node('tr'),td=node('td','Không có file khớp. Thử chọn “Tất cả file” hoặc bỏ bộ lọc nhóm.','empty');td.colSpan=4;tr.append(td);$('rows').append(tr);}$('matches').textContent=format(filtered.length)+' / '+format(all.length)+' đường dẫn';$('page').textContent=(page+1)+' / '+pages;$('prev').disabled=page===0;$('next').disabled=page>=pages-1;document.body.dataset.visibleCount=filtered.length;document.body.dataset.totalCount=all.length;document.body.dataset.renderedRows=Math.min(size,Math.max(0,filtered.length-page*size));}
function apply(){const terms=normalize($('query').value).trim().split(/\s+/).filter(Boolean),own=$('scope').value==='own',group=$('group').value;filtered=[];for(let i=0;i<all.length;i++){let r=all[i];if(own&&!['project','test','catalog'].includes(r[1]))continue;if(group!=='all'&&r[1]!==group)continue;if(!terms.every(q=>search[i].includes(q)))continue;filtered.push(i);}page=0;$('scopeNote').textContent=own?'Đang hiển thị file dự án và test. Archive, kết quả, dependency, build/cache vẫn có trong danh mục; chọn “Tất cả file” để xem.':'Đang tra toàn bộ danh mục, gồm cả file ẩn và thư viện. Symlink thư mục được liệt kê nhưng không quét lại đích.';render();}
$('total').textContent=format(all.length);const own=all.filter(r=>['project','test'].includes(r[1])).length,deps=data.counts.dependency||0;$('own').textContent=format(own);$('deps').textContent=format(deps);$('others').textContent=format(all.length-own-deps);$('stamp').textContent='Xuất lúc '+data.created+' · Offline · Dữ liệu nhúng sẵn, không dùng CDN';$('coverage').textContent='Đã kiểm kê '+format(all.length)+' file/liên kết; '+data.directoryLinks+' liên kết thư mục được liệt kê mà không đi theo; '+data.errors.length+' lỗi đọc danh mục. Kích thước là lstat của đường dẫn, không cộng lặp kích thước đích symlink. Không bỏ qua .git, .venv, archive hay runs.';$('root').textContent='Workspace: '+data.root;$('errors').textContent=data.errors.length?data.errors.join('\n'):'Không có lỗi truy cập khi kiểm kê. Nội dung nhị phân và file thư viện không được đọc để suy chức năng chuyên biệt.';
for(const [key,label] of Object.entries(labels)){const opt=node('option',label);opt.value=key;$('group').append(opt);const tr=node('tr');tr.append(node('td',label),node('td',format(data.counts[key]||0)));$('groups').append(tr);}
const lr=all.find(r=>r[0]==='rl_training/long_run.py');if(lr){const d=defs[lr[2]];$('longrun').append(node('p',d.role),node('p','Đầu vào: '+d.inputs),node('p','Đầu ra và giới hạn: '+d.outputs),detail(d));}
$('query').addEventListener('input',()=>{clearTimeout(window.searchTimer);window.searchTimer=setTimeout(apply,140);});for(const id of ['scope','group','pageSize'])$(id).addEventListener('change',apply);$('prev').onclick=()=>{page--;render();};$('next').onclick=()=>{page++;render();};for(const button of document.querySelectorAll('[data-query]'))button.onclick=()=>{$('query').value=button.dataset.query;$('scope').value='own';$('group').value='all';apply();};$('showArchive').onclick=()=>{$('scope').value='all';$('group').value='archive';$('query').value='';apply();};$('reset').onclick=()=>{$('query').value='';$('scope').value='own';$('group').value='all';apply();};
$('export').onclick=()=>{const quote=v=>'"'+String(v??'').replace(/"/g,'""')+'"';const lines=[['Đường dẫn','Nhóm','Nghĩa tên','Công dụng','Đầu vào','Đầu ra / lưu ý','Loại','Byte','Thời điểm sửa','Bằng chứng']];for(const i of filtered){const r=all[i],d=defs[r[2]];lines.push([r[0],labels[r[1]],d.meaning,d.role,d.inputs,d.outputs,r[5],r[3],r[4],d.evidence]);}const blob=new Blob(['\uFEFF'+lines.map(l=>l.map(quote).join(',')).join('\r\n')],{type:'text/csv;charset=utf-8'}),a=node('a');a.href=URL.createObjectURL(blob);a.download='danh_muc_file_dang_loc.csv';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);};
const params=new URLSearchParams(location.search);if(params.get('scope')==='all')$('scope').value='all';if(params.has('q'))$('query').value=params.get('q');if(labels[params.get('group')])$('group').value=params.get('group');apply();
</script></body></html>'''


def build(root, output):
    root, output = root.resolve(), output.resolve()
    output.relative_to(root)  # Không tạo danh mục bên ngoài cây root rồi gắn nhãn sai.
    if not output.exists():
        output.parent.mkdir(parents=True, exist_ok=True)
        output.touch()
    rows, descriptions, errors = inventory(root, output)
    payload = dict(root=str(root), created=datetime.now(ZoneInfo('Asia/Bangkok')).isoformat(timespec='seconds'),
                   groups=GROUPS, rows=rows, descriptions=descriptions,
                   counts=dict(Counter(r[1] for r in rows)), errors=errors,
                   directoryLinks=sum(r[5] == 'symlink_dir' for r in rows),
                   prefix=Path(os.path.relpath(root, output.parent)).as_posix().rstrip('/')+'/')
    # Tránh dữ liệu tên file/docstring kết thúc thẻ script; DOM dùng textContent.
    raw = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).replace('<', chr(92)+'u003c')
    output.write_text(PAGE.replace('__DATA__', raw), encoding='utf-8')
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or args.root/'DANH_MUC_FILE_REPO.html'
    payload = build(args.root, output)
    print(json.dumps(dict(output=str(output.resolve()), files=len(payload['rows']),
                          groups=payload['counts'], errors=payload['errors']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
