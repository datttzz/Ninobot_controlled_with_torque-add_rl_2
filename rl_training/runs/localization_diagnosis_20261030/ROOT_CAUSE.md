# Điều tra sai lệch định vị AMCL/TF so với Ground Truth

## Kết luận chính

Đã dừng tuning. Profile PID vận hành và ứng viên khóa giữ nguyên byte. Chỉ phân tích 16 episode cũ và chạy đúng một episode PID headless với rosbag thụ động; không train/nạp/evaluate SAC, không benchmark ba controller, không đổi dung sai.

Có bằng chứng độc lập rằng wheel odometry tích lũy sai số lớn trên gờ; map→odom đang bù phần lớn, không phải tự sinh toàn bộ sai lệch. Cấu hình thời gian TF và chất lượng scan khi thân nghiêng làm sai lệch tạm thời lớn hơn. Bản đồ có sai khác tọa độ cỡ cm so với world; identity trước đây chỉ là quy ước danh định. Chưa có bằng chứng đủ để quy riêng lượt cũ lệch 0.84 m cho AMCL nhầm mode trong hành lang.

## Hình cần mở

- [AMCL/TF−GT theo tiến độ: outlier cũ và ba lượt verification](old_registered/errors_progress.png)
- [Cùng các lượt theo thời gian](old_registered/errors_time.png)
- [Tách wheel odometry và hiệu chỉnh map→odom](odom_map_decomposition.png)
- [So sánh pose AMCL gốc, TF thực tế và xử lý thời gian offline](tf_timing_vs_raw_amcl.png)
- [Gờ, độ nghiêng thân, scan chạm sàn và odometry](bumps_scan_and_odom.png)
- [Mức phân biệt vị trí từ scan trong hành lang](hall_scan_observability.png)
- [Map so với scan/world độc lập](map_registration.png)

Mở PNG bằng trình xem ảnh. Mỗi hình có SVG đi kèm nếu cần phóng to (trừ hai hình phụ). Toàn bộ 16 lượt có trong old_registered/page_1..page_4, mỗi page gồm errors_time.png và errors_progress.png. old_nominal giữ hình và CSV theo identity để đối chiếu giả định, không xóa kết quả cũ.

## 1. Frame, điểm robot và đăng ký map/world

Pose ghi là AMCL trong telemetry cũ thực ra là TF map→base_footprint: kết hợp AMCL với odometry, không phải message /amcl_pose. Phân tích mới tách rõ ba nguồn /odom, /amcl_pose và TF map→base. GT là pose model vật lý world/base_footprint; model origin trùng base_footprint. TF static ghi thực tế xác nhận footprint→base_link chỉ z=0.0625 m, base_link→laser=(0.1,0,0.18), không quay. Đầu laser ở (.1,0,.2425) so với footprint. Cùng điểm robot được giữ khi tính sai số; không so IMU/laser origin với footprint.

**Đính chính mức chắc chắn trước đây:** spawn và initialpose cùng số (0,0,0) không đủ để chứng minh map/world là identity tới cm. Map được dựng từ SLAM, pixel 0.05 m. Kết quả cũ vẫn được giữ, nhưng kết luận “identity đã xác minh chính xác” là quá mạnh.

Đã dùng ba scan đứng yên tại 1.5/2.5/3.5 s, pose GT world độc lập và TF laser cố định để đăng ký một transform duy nhất vào map. Không dùng pose AMCL để fit, không căn chỉnh GT theo từng lượt. Các cặp tường SDF x=-2/12,y=±2 và raster cũng xác nhận sai khác cỡ một pixel.

Ước lượng cố định **map→world = (-0.054224, -0.035178 m, 0.00151335 rad)**. Xem REGISTRATION.json. Mức bất định phân giải dùng để đọc hình là khoảng 0.05 m và 0.0036 rad; đây không phải confidence interval hay hiệu chuẩn đo đạc tuyệt đối. Sai số dựng bản đồ có thể không hoàn toàn là rigid transform. Không tự đưa transform này vào TF runtime hoặc ghi đè metadata cũ.

Vì vậy các sai lệch cuối lượt chỉ vài cm còn bị confound bởi đăng ký bản đồ. So goal map (11,0) với world (11,0) cũng là một giả định cần khai báo, không tự động là cùng đích vật lý; quy ước world-goal đã dùng ở protocol cũ phải phân biệt với goal map. Không đổi kết quả pass cũ hoặc nới ngưỡng trong điều tra này. Muốn xét pass sát ngưỡng chính xác cần đăng ký/landmark tốt hơn độ phân giải hiện tại.

## 2. Có ngay từ đầu hay tích lũy?

Dưới identity, median sai lệch đầu 16 lượt chỉ khoảng (+7.0,-1.9) mm. Sau đăng ký tĩnh độc lập, median đầu khoảng (-47,-37) mm: đây là thành phần map/initialpose tĩnh, không được gọi là drift do bánh. Sai số động tăng sau khi xe chạy.

Outlier PID cũ: X estimate−GT tăng từ khoảng +0.007 lên +0.840 m dưới identity. Với cùng transform tĩnh mới, cuối còn khoảng +0.786 m; phần tăng theo thời gian vẫn +0.833 m. Một phép đổi gốc tĩnh không xóa được drift này. Nguồn gốc chi tiết của riêng lượt cũ chưa tách được vì không ghi raw odom/TF/scan.

Thông thường sai số X trong lúc chạy lớn hơn Y; sau bù static offset, sai lệch Y cuối có thể lớn hơn X. Không thể nói mọi sai số đều dọc hành lang. Yaw cuối thường nhỏ nhưng có xung lớn khi đi qua gờ; không được lấy yaw cuối để suy rằng cả lượt không lệch hướng.

## 3. Bằng chứng tách nguồn trong một lượt kiểm chứng

| Thành phần ở cuối lượt | X (m) | Y (m) | Yaw (rad) |
|---|---:|---:|---:|
| Wheel odometry−GT | 0.404814 | -2.165185 | -0.440535 |
| Hiệu ứng hiệu chỉnh map→odom | -0.382123 | 2.107789 | 0.444713 |
| TF cuối−GT | 0.022692 | -0.057396 | 0.004178 |

Phân rã Cartesian: (odom_world−GT) + (estimated_world−odom_world) = estimated_world−GT. Sai số số học tối đa 4.4e-16 m. Odometry được gắn với world từ pose khởi tạo vật lý, không từ AMCL. Trên log này wheel odom đã lệch ngang ~2.165 m và yaw ~0.4405 rad (25.2°); AMCL/map→odom bù lại phần lớn. Do đó không có bằng chứng AMCL là nguồn duy nhất hay nguồn đầu tiên tạo ra drift.

Tái tích phân encoder bằng cùng radius/track và timestamp joint_states cho endpoint chênh raw odom khoảng (3.0,10.4) mm và 0.00104 rad; RMS sai khác khoảng (2.1,6.1) mm/0.00107 rad. Độ chênh này rất nhỏ so với drift vật lý. Bằng chứng ủng hộ mô hình lăn từ bánh không phản ánh đủ chuyển động thân qua gờ; chưa phân biệt trượt, mất tải/nhấc bánh, lực tiếp xúc caster hoặc thành phần 3D. Không tìm được lỗi tích phân đơn giản đủ để giải thích drift mét.

Wheel radius config và collision đều 0.0625 m. Track config 0.34273666 m lấy theo CAD COM; hai tâm cylinder collision cách 0.340 m: lệch 0.805%. Thay riêng track bằng 0.340 sẽ chỉ làm yaw tích phân của lượt này đổi khoảng -0.00556 rad, nhỏ xa 0.4405 rad. Có bất nhất hình học cần ghi nhận, nhưng chưa có dữ liệu contact để xem effective rolling track thực bằng tâm hình học; không tự sửa tham số để bù trượt.

## 4. Gờ, đoạn phẳng và scan

Khi xe tiến >0.05 m/s, độ biến thiên tuyệt đối yaw-error odom trên mỗi mét: vùng gờ/caster 0.2842 rad/m, ngoài cửa sổ 0.0316 rad/m, tỷ số 9.00. Cửa sổ mô tả là base hoặc caster trước cách tâm dây ±0.15 m, không phải cảm biến xác nhận va chạm. Dữ liệu chậm/quay tại chỗ được giữ thành hàng riêng, không loại khỏi phân tích tổng.

Sai số hướng phát sinh mạnh gần gờ; sau đó sai hướng còn lại làm tích phân vị trí tiếp tục lệch trên đoạn phẳng. Không thể gán mọi thay đổi Y trên đoạn phẳng thành một sự kiện trượt mới. Xem odom_bump_vs_other.csv và đồ thị bumps_scan_and_odom.png.

67/692 scan trong lượt có endpoint gần mặt sàn theo phép chiếu 3D GT; cao nhất 38.9% endpoint, trùng các đoạn thân nghiêng. Map 2D chỉ biểu diễn tường/sàn phẳng dưới dạng occupancy, nên những scan đó không phù hợp hoàn toàn giả thiết laser phẳng. Đây là tác nhân khả dĩ ảnh hưởng hiệu chỉnh, chưa phải phép phân lập nguyên nhân duy nhất. [Code AMCL giả thiết roll/pitch laser bằng zero và cập nhật TF](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp).

## 5. Hành lang có mơ hồ không?

Đánh giá offline các lát điểm số likelihood_field từ scan thực, cùng sigma_hit/z_hit/z_rand/max_beams hiện tại. Bề rộng vùng >=95% điểm số cực đại theo X khoảng 0.14–0.185 m; theo Y khoảng 0.0925–0.095 m. Đây là bề rộng score, KHÔNG phải khoảng tin cậy vị trí. Hai hướng đều có đỉnh; định vị dọc yếu hơn khoảng 1.5–2 lần trong các scan kiểm tra.

Khoảng 19–29% endpoint laser hữu hạn nằm gần hai tường cuối (phần lớn còn lại ở tường dài). Hành lang này không giống hành lang vô hạn chỉ có hai tường song song. Chưa thấy bằng chứng đa nghiệm cách nhau ~0.8 m trong các lát score đã kiểm tra; không kết luận “AMCL chắc chắn nhầm vị trí dọc hall” cho outlier cũ. Particle cloud mới đã ghi nhưng không khôi phục được posterior của lượt cũ.

Phép tính score dùng dạng 1+sum(p_z^3) từ [likelihood field của Nav2 Jazzy](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp), với distance field offline của raster; không phải tái chạy đầy đủ particle filter hoặc marginal posterior.

## 6. Timestamp và hiệu ứng TF

Clock/odom/GT liên tục; GT gap tối đa 0.01 s, odom TF khoảng 0.023 s. Độ lệch nhận so với /clock gần nhất: scan P95 ~0.011 s, AMCL pose P95 ~0.012 s. Đây là số đo thứ tự nhận trong rosbag, không phải đồng hồ mạng đã hiệu chuẩn. /initialpose có header zero nên không dùng để suy độ trễ cảm biến.

TF map→odom có header đúng scan_stamp+1.0 s, phù hợp transform_tolerance=1.0 hiện tại. Đây là cách postdate có chủ ý của AMCL, không phải scan bị trễ 1 giây. Tuy nhiên nội suy TF tại timestamp odom khiến correction thay đổi có tác dụng muộn; đã tái dựng và khớp pose TF thật logger đọc tới ~1.3e-14 đơn vị số học, nên không phải lỗi của biểu đồ.

Xử lý offline cùng odom/GT nhưng dùng correction đã nhận mới nhất (không lấy dữ liệu tương lai) cho RMS yaw-error khoảng 0.0786 rad, so với 0.1133 rad ở chuỗi TF gốc; RMS X từ 0.0930 xuống 0.0781 m. Điều này xác nhận timing là một thành phần khuếch đại sai số tạm thời. Đây KHÔNG phải kết quả điều khiển vòng kín sau sửa config, không đảm bảo đổi transform_tolerance sẽ làm robot thực chạy tốt hơn.

## 7. Bản sửa tối thiểu và phần chưa biết

Đã sửa phương pháp chẩn đoán: đồng bộ hai pose về cùng timestamp, wrap yaw đúng, tách TF estimate khỏi /amcl_pose thật; dùng một registration độc lập cố định và công khai bất định. Code mới localization_math.py/localization_diagnosis.py; 5 test có đáp án biết trước đạt (SE2, timestamp/gap/duplicate, góc ±pi và biên pixel). Không chỉnh code controller, Nav2, wheel geometry hoặc YAML PID. Không có lỗi runtime đơn lẻ đã phân lập đủ chắc để áp dụng một “fix” cấu hình rồi tuyên bố đã chữa drift.

Những dữ liệu còn thiếu để chốt cơ chế hoặc sửa runtime:

- Raw /odom, /tf, /amcl_pose, /scan, particle cloud của chính outlier cũ 0.84 m: chưa từng ghi, không thể suy lại chính xác từ telemetry 50 Hz.
- Contact state/force, tốc độ tiếp xúc hai bánh/caster so mặt đất: cần để phân biệt trượt, mất tải và hình học effective track. Không chạy thêm mô phỏng trong nhiệm vụ này.
- Map provenance/landmark đo độc lập hoặc map hình học chuẩn để khóa map↔world chính xác hơn cm và phân biệt offset cứng với méo SLAM.
- Một phép kiểm chứng vòng kín riêng nếu muốn thử thay timing TF; điều tra này đã dùng đúng một episode nên chưa áp dụng/chạy thêm thử cấu hình.

Mọi profile và config bảo vệ khớp BEFORE.json; VALIDATION.json ghi kiểm tra byte, hash raw logs, đúng một episode và không còn Gazebo/job hoạt động. Lỗi CLI recorder ban đầu và bản phân tích trung gian có lỗi biên pixel được giữ nguyên để truy vết; chúng không được tính thành thất bại điều khiển.

## Chạy lại phân tích offline trên bag đã lưu

```bash
cd /home/tiendat/project/Ninobot_controlled_with_torque-add_rl_2
source /opt/ros/jazzy/setup.bash
source rl_training/.venv/bin/activate
python rl_training/localization_diagnosis.py --output-dir rl_training/runs/localization_diagnosis_20261030
```
Lệnh chỉ tính lại các file chẩn đoán trong thư mục này; không mở Gazebo, không chạy PID/SAC, không sửa raw bag hay log gốc.
