# Chạy thử Nino và RL trên laptop Ubuntu

## 1. Phạm vi hướng dẫn

Dự án đang nằm tại:

```text
/home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
```

Hướng dẫn này dành cho Ubuntu trực tiếp trên laptop có ROS 2 Jazzy và RTX 4060.
Các lệnh bên dưới không dùng Docker/WSLg và không sử dụng đường dẫn `/ros2_ws`
của container cũ.

Kết quả kiểm tra ban đầu trên laptop:

- Ba package ROS build thành công trong thư mục kiểm tra tạm.
- Gazebo chạy được ở chế độ server, robot khởi tạo được.
- `joint_state_broadcaster` và `wheel_effort_controller` đều `active`.
- Đã nhận được `/odom`.
- Chưa kiểm chứng đầy đủ Nav2, baseline và huấn luyện RL trên laptop.

Thực hiện theo thứ tự: mô phỏng → kiểm tra chuyển động → cài môi trường RL →
baseline → kiểm tra SAC ngắn → train ngắn → đánh giá. Chỉ chuyển bước khi bước
trước đã chạy được hoặc đã hiểu rõ nguyên nhân thất bại.

## 2. Phân biệt với bài PID trước đó

Dự án này dùng luồng:

```text
Nav2 → vận tốc → PI bánh xe → mô-men nền ─┐
                                       ├→ giới hạn → bánh xe trong Gazebo
SAC → mô-men bù cho bánh trái/phải ──────┘
```

- **Baseline:** chạy điều khiển nền, phần bù RL bằng 0.
- **Episode:** một lượt khởi tạo mô phỏng, định vị, lập đường và chạy tới đích.
- **SAC:** thuật toán học tăng cường tạo phần bù mô-men cho hai bánh.
- **Smoke:** lượt kiểm tra ngắn để xác nhận có thu dữ liệu và cập nhật mạng.

Đây không phải launch `pid_terrain_benchmark`. Trong quy trình RL bên dưới,
script tự mở Gazebo/Nav2 và gửi đích. Không dùng `/start_tracking` hoặc vừa chạy
PID cũ vừa chạy baseline/RL.

## 3. Chuẩn bị terminal

Dừng các phiên Gazebo, Nav2, PID, teleop và lệnh phát `/cmd_vel` đang chạy trước.
Dùng `Ctrl+C` tại terminal đã khởi động từng phiên; không cần tắt hàng loạt
các tiến trình bằng `killall`.

Mở hai terminal mới trong VS Code:

| Terminal | Dùng để làm gì |
|---|---|
| Terminal 1 | Build, chạy mô phỏng; sau đó dùng để chạy baseline hoặc RL |
| Terminal 2 | Kiểm tra node/topic, điều khiển thử, xem kết quả |

Nếu có `(base)` hoặc môi trường Conda khác, thoát bằng:

```bash
conda deactivate
```

Nếu đang lồng nhiều môi trường, thoát tiếp cho đến khi không còn dùng Python
Conda. Không source workspace `ros2_ws` của bài PID cũ vào các terminal này.

Dùng domain 78 và một Gazebo partition riêng cho dự án mới. Các terminal liên
quan phải dùng cùng giá trị:

```bash
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_laptop_rl
```

Domain/partition giúp tách thông tin giữa các phiên, nhưng vẫn nên dừng phiên
mô phỏng khác để giảm tải và tránh mở nhầm robot.

## 4. Terminal 1: build tại thư mục dự án

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

Cần thấy ba package hoàn thành: `nino_control`, `nino_description`,
`nino_navigation`. Nếu build lỗi, xem lỗi đầu tiên trước khi chạy tiếp.

Kiểm tra đang lấy package từ đúng dự án:

```bash
ros2 pkg prefix nino_description
ros2 pkg prefix nino_control
ros2 pkg prefix nino_navigation
```

Đường dẫn kết quả phải nằm dưới thư mục `install` của dự án này. Thư mục
`/tmp/nino_portability_check` dùng trong lần kiểm tra trước không phải vùng
cài đặt chính để dùng lâu dài.

## 5. Terminal 1: mở Gazebo và RViz để kiểm tra robot

```bash
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_laptop_rl

ros2 launch nino_description sim.launch.py \
  rviz:=true \
  sensor_monitor:=true
```

Giữ terminal này chạy. Đợi robot xuất hiện và controller được kích hoạt.
Mặc định phần bù RL chưa được bật trong launch mô phỏng này.

Gazebo tính chuyển động và va chạm; RViz hiển thị robot cùng dữ liệu ROS.
Mở cả hai là bình thường.

## 6. Terminal 2: kiểm tra dữ liệu và chuyển động

Nạp môi trường cho terminal mới:

```bash
source /opt/ros/jazzy/setup.bash
source /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/install/setup.bash
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_laptop_rl
```

### 6.1. Kiểm tra node và controller

```bash
ros2 node list --no-daemon --spin-time 3
ros2 control list_controllers
```

Cần thấy `/controller_manager`, `/effort_drive`, `/robot_state_publisher`,
`/ros_gz_bridge` và các node controller. Hai controller sau cần `active`:

```text
joint_state_broadcaster
wheel_effort_controller
```

Nếu lệnh liệt kê thông thường chỉ thấy một phần node nhưng `--no-daemon`
thấy đủ, khởi động lại daemon trong terminal 2:

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

Thao tác này không tắt robot hay Gazebo.

### 6.2. Kiểm tra các topic

Chạy từng lệnh, đợi vài giây rồi nhấn `Ctrl+C` trước khi chạy lệnh tiếp theo:

```bash
ros2 topic hz /odom
```

```bash
ros2 topic hz /joint_states
```

```bash
ros2 topic hz /imu/data
```

```bash
ros2 topic hz /scan
```

Cấu hình đặt odometry/IMU khoảng 50 Hz, lidar khoảng 10 Hz. Tần số quan sát
có thể thấp hơn khi máy chạy mô phỏng chậm. Đặc biệt chú ý bản tin có phát
liên tục, không chỉ có tên topic.

Kiểm tra đồng hồ, lệnh tự thoát sau một bản tin:

```bash
ros2 topic echo /clock --once
```

Pipeline hiện tại dùng TF map→base_footprint cho pose ước lượng AMCL, `/imu/data` cho IMU, `/joint_states` cho encoder và `/odom` cho vận tốc ước lượng. Mô-men ghi từ lệnh controller, không phải cảm biến lực.

### 6.3. Cho xe chạy chậm

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Quan sát vài giây, nhấn `Ctrl+C`, rồi gửi lệnh dừng:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Nếu xe chuyển động và dừng được, phần mô phỏng/truyền động đã qua kiểm tra cơ
bản. Bài này chưa kiểm tra khả năng bám đường hoặc hiệu quả RL.

**Kết thúc bước kiểm tra thủ công:** ở terminal 1 nhấn `Ctrl+C`, đợi Gazebo
và các node đóng xong. Dừng mọi lệnh phát vận tốc ở terminal 2.

## 7. Terminal 1: tạo môi trường Python cho RL

Script baseline cũng import thư viện SAC/PyTorch, nên cần bước cài đặt này
trước cả khi chỉ chạy baseline không bù RL.

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

/usr/bin/python3 -m venv --system-site-packages rl_training/.venv
source rl_training/.venv/bin/activate

python -m pip install --upgrade pip
python -m pip install torch==2.7.1 \
  --index-url https://download.pytorch.org/whl/cu128
python -m pip install -r rl_training/requirements.txt
```

Các phiên bản trên theo cấu hình của dự án. Việc tải PyTorch/CUDA có thể cần
nhiều dung lượng và thời gian. `--system-site-packages` giúp môi trường mới
nhìn thấy `rclpy` và các thư viện ROS đã cài cho Python hệ thống.

Nếu lệnh tạo venv báo thiếu `ensurepip`, cài gói hỗ trợ rồi tạo lại:

```bash
sudo apt install python3-venv
```

Kiểm tra môi trường:

```bash
which python
python - <<'PY'
import sys
sys.path.insert(0, 'rl_training')
from plotting_compat import prepare_matplotlib_namespace
prepare_matplotlib_namespace()
import rclpy, torch, gymnasium, stable_baselines3
from mpl_toolkits.mplot3d import Axes3D
print('Import ROS, RL và vẽ 3D thành công')
PY
python -m pip check
```

`which python` cần trỏ vào `rl_training/.venv/bin/python`. Nếu `pip check`
báo xung đột, xử lý theo thông báo trước khi train; không tự nâng hàng loạt
các phiên bản đã ghim trong requirements.

Nếu `pip check` báo `pynacl ... requires cffi, which is not installed`, cài
dependency còn thiếu trong venv rồi kiểm tra lại:

```bash
python -m pip install --no-cache-dir cffi
python -m pip check
```

Nếu import trực tiếp SB3 báo `Unable to import Axes3D`, đó là xung đột
namespace Matplotlib giữa Ubuntu và venv. Đoạn kiểm tra trên gọi hàm
`prepare_matplotlib_namespace()` có sẵn trong repo trước khi import SB3.
`train.py` cũng đã gọi hàm này. Không cần gỡ Matplotlib hệ thống để xử lý.

## 8. Terminal 1: kiểm tra GPU tính toán thật

```bash
nvidia-smi
```

Sau đó, trong venv vừa kích hoạt:

```bash
python - <<'PY'
import torch

print('PyTorch:', torch.__version__)
print('CUDA của PyTorch:', torch.version.cuda)
print('CUDA sẵn sàng:', torch.cuda.is_available())
assert torch.cuda.is_available(), 'Chưa sử dụng được CUDA'
print('GPU:', torch.cuda.get_device_name(0))
x = torch.ones((16, 16), device='cuda', requires_grad=True)
(x @ x).sum().backward()
torch.cuda.synchronize()
print('Tính xuôi và lan truyền ngược trên GPU thành công')
PY
```

Chỉ thấy tên GPU trong `nvidia-smi` chưa đủ xác nhận PyTorch tính được trên GPU.
Nếu bài kiểm tra CUDA lỗi, chưa dùng `--device cuda` để train. Có thể kiểm tra
luồng bằng `--device cpu` trước, nhưng không coi đó là xác nhận GPU đã hoạt động.

## 9. Terminal 1: nạp môi trường trước mỗi phiên baseline/RL

Mỗi khi mở terminal mới để chạy script, dùng đủ nhóm lệnh:

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source rl_training/.venv/bin/activate
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=nino_laptop_rl
```

Không để launch thủ công ở bước 5 còn chạy. Các script tiếp theo tự mở
Gazebo và Nav2, đặt vị trí định vị, gửi đích và đóng các tiến trình chúng tạo.
Không mở thêm launch mô phỏng/Nav2 cùng lúc, không tự gửi `/cmd_vel`.

## 10. Terminal 1: chạy baseline ngắn trước

Baseline dùng phần bù RL bằng 0, để đo hoạt động của điều khiển nền.
Thử đích gần 3 m trước:

```bash
python rl_training/train.py \
  --mode baseline \
  --map /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/maps/hall_01.yaml \
  --goal-x 3.0 \
  --max-episode-seconds 60 \
  --episodes 1 \
  --gui
```

`--gui` mở cửa sổ Gazebo. Script không tự mở RViz trong bài này. Đợi script
kết thúc và xem dòng kết quả hoặc file log nếu gặp lỗi. Giới hạn episode tính
bằng thời gian mô phỏng; khởi tạo, reset và thời gian chờ làm tổng thời gian
trên máy dài hơn con số đó.

Phân biệt:

- **Có outcome thành công/thất bại/hết giờ:** episode đã chạy và có kết quả;
  thất bại vượt gờ vẫn là dữ liệu cần đánh giá.
- **Lỗi ROS, Nav2, cảm biến hoặc timeout khởi tạo:** cần sửa hạ tầng trước,
  không coi là bằng chứng policy điều khiển kém.

Nếu bài ngắn chạy được, thử baseline đầy đủ với đích mặc định x=11 m:

```bash
python rl_training/train.py \
  --mode baseline \
  --map /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/maps/hall_01.yaml \
  --episodes 3
```

Lệnh này mặc định chỉ mở server mô phỏng. Thêm `--gui` nếu muốn nhìn xe.
Mỗi episode khởi động lại các thành phần để đưa hệ thống về trạng thái mới.

## 11. Terminal 1: kiểm tra SAC ngắn

Sau khi baseline không còn lỗi hạ tầng và CUDA đã qua kiểm tra:

```bash
python rl_training/train.py \
  --mode smoke \
  --map /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/maps/hall_01.yaml \
  --steps 32 \
  --device cuda
```

Smoke dùng mạng nhỏ, bắt đầu cập nhật sau 8 bước, giới hạn episode 2 giây và
có thể khởi động lại mô phỏng nhiều lần. Nó kiểm tra đường dữ liệu và cập nhật
mạng SAC, không chứng minh robot đã học cách vượt gờ.

Cần thấy thông tin thiết bị và kết quả có `gradient_updates` lớn hơn 0.
Xem thêm `device_verification.json` trong thư mục lượt chạy.

Nếu chỉ muốn kiểm tra bằng CPU, đổi thành `--device cpu`. Đây là lựa chọn
chủ động, không phải sửa cấu hình driver hay Docker.

## 12. Terminal 1: chọn số step và train theo từng chặng

### 12.1. Train thử ban đầu

Ví dụ 2.000 bước, đủ để vượt giai đoạn thu thập ban đầu 1.000 bước của cấu
hình train mặc định và bắt đầu cập nhật mạng:

```bash
python rl_training/train.py \
  --mode train \
  --map /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/maps/hall_01.yaml \
  --steps 2000 \
  --seed 1 \
  --device cuda
```

Đây là lượt train thử, chưa đủ để khẳng định policy tốt hơn baseline.
Thời gian phụ thuộc tốc độ mô phỏng, khởi tạo Nav2 và số lần reset. GPU xử lý
mạng, trong khi nhiều phần của Gazebo/ROS vẫn phụ thuộc CPU.

Muốn dừng, nhấn `Ctrl+C` một lần rồi đợi script lưu model có sẵn và đóng các
tiến trình nó tạo. Không khởi chạy ngay job 50 triệu bước theo ví dụ trên máy
cũ. Chưa có kiểm chứng thời gian, dung lượng lưu trữ và hiệu quả trên laptop.

### 12.2. Step, episode và số lần train khác nhau thế nào?

- **Step:** một lần môi trường nhận action, tiến mô phỏng và trả observation/reward.
  Cấu hình hiện tại đặt khoảng 0.1 giây mô phỏng mỗi step; thời gian thực có thể
  dài hơn vì còn cập nhật mạng, chờ ROS và khởi động lại các episode.
- **Episode:** một lượt robot chạy tới đích, thất bại hoặc hết giờ. Độ dài không
  cố định, nên 10.000 step không đồng nghĩa với 10.000 lượt chạy xe.
- **Seed:** giá trị khởi tạo ngẫu nhiên. Train lại từ đầu với seed khác tạo một
  lượt học độc lập, giúp kiểm tra kết quả có phụ thuộc vào một lần may mắn hay không.
- `--steps` quyết định số bước train; `--episodes` dùng cho baseline/evaluate.
  Không dùng `--episodes` để yêu cầu số lượt trong chế độ train hiện tại.

### 12.3. Train mới với số step tùy chọn

Trong terminal 1 đã nạp ROS, workspace và venv theo bước 9, đặt số bước muốn chạy:

```bash
STEPS=10000
CHECKPOINT_EVERY=5000

python rl_training/train.py \
  --mode train \
  --map /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/maps/hall_01.yaml \
  --steps "$STEPS" \
  --checkpoint-every "$CHECKPOINT_EVERY" \
  --seed 1 \
  --device cuda
```

Thay `STEPS` bằng số nguyên dương như `2000`, `10000`, `20000` hoặc `50000`.
Lệnh không có `--resume` sẽ tạo model mới từ đầu và thư mục run mới. Nó không
học tiếp model của lượt trước dù bạn chạy trong cùng terminal.

`--checkpoint-every 5000` lưu model và replay buffer sau mỗi 5.000 bước train.
Nếu tổng bước dưới 5.000, không có checkpoint định kỳ nhưng cuối lượt bình
thường vẫn lưu `model.zip` và `replay_buffer.pkl`. Không đặt checkpoint quá dày
chỉ để theo dõi: mỗi checkpoint kèm replay buffer là một bản sao dữ liệu.

Với cấu hình chuẩn, `learning_starts=1000`: 1.000 bước đầu chủ yếu thu kinh nghiệm.
Train đúng 1.000 bước chưa đủ để chứng minh mạng đã cập nhật; dùng 2.000 bước
cho bài kiểm tra train ban đầu. Chế độ smoke có cấu hình riêng cho kiểm tra ngắn.

### 12.4. Mức chạy gợi ý cho laptop hiện tại

Laptop đã kiểm tra có i7-13650HX, RTX 4060 Laptop 8 GB và khoảng 24 GB RAM.
Run `train_20261026_173506_446593` đã chạy 2.000 bước trong khoảng **5 phút
49 giây**, ước lượng từ thời điểm ghi metadata đến kết quả cuối.

Bảng thời gian dưới đây chỉ là phép nhân tuyến tính từ lượt 2.000 bước đó,
không phải số đo của các lượt dài. Tải máy, số lần reset, tỷ lệ thành công,
giai đoạn bắt đầu học và nhiệt độ có thể làm tốc độ khác nhiều.

| Tổng step mục tiêu | Mục đích gợi ý | Thời gian tuyến tính tham khảo | Checkpoint gợi ý |
|---:|---|---|---:|
| 2.000 | Xác nhận train có cập nhật mạng | Đã đo khoảng 6 phút | 2.000–5.000 |
| 10.000 | Mốc tiếp theo nên thử trên laptop | Khoảng 30 phút | 5.000 |
| 20.000 | Đánh giá sau khi mốc 10.000 ổn | Khoảng 1 giờ | 5.000 |
| 50.000 | Thử học dài hơn nếu kết quả có tiến triển | Khoảng 2 giờ 25 phút | 10.000 |
| 100.000 | Chỉ chạy khi các mốc trước ổn định và có đủ dung lượng | Khoảng 4 giờ 50 phút | 10.000–20.000 |

Gợi ý thực hiện:

1. Với model 2.000 bước đang có, tiếp tục tới **10.000 bước**, đánh giá 3 lượt.
2. Nếu không có lỗi hạ tầng, tiếp tục tới **20.000**, đánh giá lại cùng điều kiện.
3. Chỉ cân nhắc **50.000** hoặc **100.000** khi các mốc trước cho dữ liệu hữu ích.
4. Nếu kết quả không cải thiện, xem lỗi bám đường, reward và giới hạn điều khiển
   trước; không mặc định tăng số step sẽ làm model tốt hơn.

Đây là lộ trình thử, không phải yêu cầu tối thiểu để robot học thành công.
Chưa có đủ kết quả để biết cần bao nhiêu step cho khả năng vượt gờ tốt trên dự án này.

### 12.5. Tiếp tục model 2.000 bước tới tổng 10.000 bước

Dùng đồng thời model, replay buffer và cấu hình của cùng run:

```bash
PREVIOUS_RUN=/home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/rl_training/runs/train_20261026_173506_446593
TARGET_STEPS=10000

python rl_training/train.py \
  --mode train \
  --config "$PREVIOUS_RUN/config.yaml" \
  --map /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/maps/hall_01.yaml \
  --resume "$PREVIOUS_RUN/model.zip" \
  --replay-buffer "$PREVIOUS_RUN/replay_buffer.pkl" \
  --steps "$TARGET_STEPS" \
  --checkpoint-every 5000 \
  --seed 1 \
  --device cuda
```

**Khi resume, `--steps` là tổng số bước đích, không phải số bước cộng thêm.**
Ví dụ từ 2.000 tới 10.000 nghĩa là học thêm 8.000 bước. Script sẽ in tương tự:

```text
Target: 10000; restored: 2000; remaining: 8000
```

Run cũ được giữ nguyên. Model sau khi train tiếp được lưu trong thư mục `Run:`
mới. Muốn tiếp tục tới 20.000, đặt `PREVIOUS_RUN` sang run mới nhất vừa hoàn
thành và `TARGET_STEPS=20000`; không tiếp tục trỏ về model 2.000 bước cũ.

Không ghép model và replay buffer của hai lượt khác nhau. Resume yêu cầu cấu
hình không thay đổi; dùng `--config` của run đã lưu. Không dùng model smoke
để nối sang bài train chuẩn vì cấu hình mạng và môi trường smoke khác.

### 12.6. Dừng giữa chừng và tiếp tục từ checkpoint

Nhấn `Ctrl+C` một lần rồi đợi script lưu và đóng mô phỏng. Nếu đã tạo model,
run có thể có `interrupted_model.zip` và `interrupted_replay.pkl`.
Khi đó thay hai đường dẫn ở lệnh resume bằng cặp file này; vẫn giữ
`--config` của chính run đó.

Nếu cần khôi phục từ checkpoint, liệt kê:

```bash
ls "$PREVIOUS_RUN/checkpoints"
```

Chọn model và replay buffer có **cùng số step**. Ví dụ:

```text
sac_5000_steps.zip
sac_replay_buffer_5000_steps.pkl
```

Dùng model checkpoint cho `--resume`, replay cùng mốc cho `--replay-buffer`,
và `--config "$PREVIOUS_RUN/config.yaml"`. Tổng bước đích phải ít nhất bằng
số bước đã lưu. Đóng cửa sổ terminal hoặc tắt nguồn đột ngột có thể không cho
script kịp lưu; checkpoint định kỳ giúp giảm lượng công việc phải chạy lại.

### 12.7. Nên chạy bao nhiêu lượt đánh giá và bao nhiêu seed?

| Giai đoạn | Train | Baseline/evaluate gợi ý |
|---|---|---|
| Kiểm tra luồng | 1 model, 1 seed | 1 episode để phát hiện lỗi hạ tầng |
| So sánh ban đầu | 1 model tại mỗi mốc step | 3 episode mỗi chế độ, tạo báo cáo như ảnh |
| Chọn model triển vọng | Đánh giá các model ở cùng điều kiện | 5–10 episode mỗi chế độ |
| Muốn kết luận chắc hơn | 3 lượt train độc lập, ví dụ seed 1, 2, 3 | Ít nhất 5 episode/model và baseline đối ứng |

Bắt đầu với **1 seed và 3 episode đánh giá** là phù hợp để thử trên laptop;
chưa cần chạy đồng thời nhiều model. Nếu so sánh seed khác, baseline và
model evaluate phải dùng cùng seed bắt đầu; script báo cáo hiện kiểm tra điều
kiện này. Ba lượt đánh giá của một model không tương đương ba lượt train
độc lập. Không chỉ chọn seed hoặc episode có kết quả đẹp để đưa vào báo cáo.

### 12.8. Theo dõi dung lượng và tải máy

Tại thời điểm soạn bổ sung này, phân vùng `/` còn khoảng 11 GB. Đây là số đo
hiện tại, cần kiểm tra lại trước các lượt dài:

```bash
df -h /
du -sh rl_training/runs
nvidia-smi
```

Replay buffer của run 2.000 bước hiện khoảng 20 MB; nhiều checkpoint tạo nhiều
bản sao, còn `steps.jsonl` tiếp tục tăng theo thời gian. Dung lượng không chỉ
phụ thuộc file model nhỏ. Giữ lại các kết quả cần đối chiếu và không tự xóa
checkpoint/run cũ chỉ vì thư mục lớn.

Gazebo và Nav2 còn dùng CPU nên GPU chưa đầy tải không đồng nghĩa train bị
lỗi. Để thử dài, dùng chế độ không GUI mặc định và chỉ chạy một phiên.
Các hướng dẫn này không thay cấu hình nguồn điện, driver, GPU hay giới hạn
mô-men của robot.

## 13. Xem kết quả và đánh giá model

Kết quả dự án này nằm trong:

```text
/home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2/rl_training/runs/
```

Không nằm trong `pid_terrain_benchmark/results` của bài PID cũ.
Terminal sẽ in `Run: ...` cho biết chính xác thư mục mới tạo.

| File/thư mục | Nội dung |
|---|---|
| `metadata.json` | Chế độ chạy, seed, thiết bị và thông tin đầu vào |
| `config.yaml` | Cấu hình đã dùng cho lượt đó |
| `result.json` | Kết quả episode hoặc số bước/cập nhật mạng |
| `device_verification.json` | Bằng chứng thiết bị sử dụng trong train/smoke |
| `episode_.../simulation.log` | Log Gazebo và điều khiển |
| `episode_.../navigation.log` | Log Nav2 và định vị |
| `episode_.../steps.jsonl` | Các bản ghi theo bước trong episode |
| `model.zip` | Model SAC khi lượt train/smoke kết thúc bình thường |
| `replay_buffer.pkl` | Dữ liệu kinh nghiệm để tiếp tục train |
| `infrastructure_fault.json` hoặc `exception.json` | Nguyên nhân lỗi nếu lượt chạy bị ngắt vì lỗi |

Liệt kê các lượt chạy trong terminal 2:

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
ls -lt rl_training/runs
find rl_training/runs -maxdepth 2 -name model.zip
```

Để chọn model và tạo bảng so sánh, thực hiện **mục 15**. Phần đó giải thích
chi tiết cách đọc tên/thời gian run, xác nhận model bao nhiêu step, thay đường
dẫn, chạy baseline/evaluate và mở báo cáo. Tránh dùng nhầm ví dụ model 2.000
bước cho lần đánh giá model 50.000 bước mới hơn.

Bản sao hiện tại chưa thấy model/replay buffer cũ. Muốn tiếp tục train cũ,
cần chép cả model, replay buffer và cấu hình/metadata đi kèm. Không tạo file
model giả hoặc dùng model smoke thay cho checkpoint cũ.

## 14. Lỗi thường gặp

| Hiện tượng | Xử lý |
|---|---|
| `ros2 pkg prefix` trỏ sang workspace PID hoặc thư mục tạm | Mở terminal mới, chỉ source ROS nền và workspace này |
| Build dùng Python Anaconda, thiếu `catkin_pkg` | Thoát Conda; dùng `/usr/bin/python3` và CMake argument ở bước 4 |
| Không import được `rclpy` trong venv | Kiểm tra venv tạo từ Python hệ thống với `--system-site-packages`, và đã source ROS |
| Không import được Torch/SB3/Gymnasium | Kiểm tra `which python`, kích hoạt đúng venv và hoàn tất bước 7 |
| CUDA không sẵn sàng hoặc phép tính CUDA lỗi | Kiểm tra PyTorch và driver; chưa train GPU trước khi bước 8 thành công |
| Chỉ thấy RViz trong `ros2 node list` | Dùng `--no-daemon`, kiểm tra domain 78 và môi trường terminal |
| Báo còn publisher `/clock` hoặc `/cmd_vel` | Dừng mô phỏng/teleop cũ trong cùng domain; script không tự tắt node ngoài |
| AMCL/Nav2 hoặc cảm biến chưa sẵn sàng | Xem `episode_.../navigation.log` và `simulation.log`; không bỏ qua kiểm tra khởi tạo |
| Không tìm thấy map | Dùng đường dẫn đầy đủ tới `maps/hall_01.yaml`; giữ file PGM cùng thư mục |
| Muốn dùng Docker theo compose cũ | Compose đang dành cho WSL2; hướng dẫn này chạy Ubuntu trực tiếp, không dùng nguyên compose đó |
| Xe kẹt hoặc episode thất bại | Lưu log để phân tích; không tăng mô-men/đổi URDF chỉ để làm bài test báo thành công |

Không thay cấu hình GPU/WSLg, URDF hoặc các giới hạn controller trong quá trình
làm theo hướng dẫn này. Trước hết xác nhận luồng chạy đúng, sau đó mới thiết
kế bài so sánh điều khiển.

## 15. Đánh giá ba controller hiện hành

Xem [rl_training/ABLATION_SUITE.md](rl_training/ABLATION_SUITE.md). Hướng dẫn này tách lệnh chạy xe (một SAC checkpoint) và lệnh phân tích ba thư mục PI/PID/PI_SAC, có GUI/headless, giới hạn tài nguyên, tuning và resume.

Phần hướng dẫn so sánh trước đây được giữ nguyên tại [archive/evaluation_legacy_20261030/HUONG_DAN_CHAY_TREN_LAPTOP.md](archive/evaluation_legacy_20261030/HUONG_DAN_CHAY_TREN_LAPTOP.md). Dữ liệu, checkpoint và báo cáo cũ không bị xóa.
