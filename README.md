> **Hệ thống giao tiếp Người - Robot phi ngôn ngữ (Human–Robot Interaction - HRI) và điều khiển robot tự hành TurtleBot 4 hoàn toàn bằng thị giác máy tính và cử chỉ bàn tay (AI Vision) theo thời gian thực.**

---
## Cấu Trúc Thư Mục Dự Án (Project Structure)

```
Turtlebot_4_cuchitay/
├── README.md                                          # Tài liệu hướng dẫn dự án (GitHub Frontpage)
├── BAO_CAO_HE_THONG_DIEU_KHIEN_CU_CHI_TAY_TURTLEBOT4.docx # Báo cáo kỹ thuật chi tiết định dạng Word
├── run_all.sh                                         # Script menu tự động chạy toàn bộ hệ thống
├── run_simulation.sh                                  # Script khởi chạy Gazebo Server & 3D GUI
├── run_gesture_control.sh                             # Script khởi chạy các Node ROS 2
├── run_camera_windows.bat                             # Script chạy nhanh Camera Streamer trên Windows
├── windows_camera_bridge.py                           # Cầu nối Video HTTP MJPEG siêu tốc (Auto-Brightness)
└── turtlebot4_gesture_control/                        # Gói ROS 2 chính
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── launch/
    │   └── gesture_control.launch.py                  # Launch file khởi động đồng thời 2 ROS nodes
    ├── models/
    │   └── gesture_recognizer.task                    # Mô hình AI Google MediaPipe chính thức
    └── turtlebot4_gesture_control/
        ├── __init__.py
        ├── gesture_recognition_node.py                # Node thu nhận ảnh, lọc nhiễu & vẽ HUD
        ├── gesture_classifier.py                      # Động cơ phân loại cử chỉ Multi-Modal AI
        └── robot_behavior_node.py                     # Node điều khiển FSM & đồng bộ Simulation Clock
```

---

## Kiến trúc Hệ thống (System Architecture)

```
+-------------------------------------------------------------------------------------------------+
|  TẦNG 1: THU NHẬN HÌNH ẢNH PHẦN CỨNG (WINDOWS HOST ENVIRONMENT)                                |
|  [ Laptop Webcam 0 ] ──► [ windows_camera_bridge.py ] (Auto-Brightness & Contrast Boost)        |
|                                     │                                                           |
|                                     ▼ Giao thức mạng cục bộ: HTTP MJPEG Stream (Port 8080)     |
+-------------------------------------┼-----------------------------------------------------------+
                                      │ (Mạng ảo nội bộ Host-to-WSL2: 172.x.x.1:8080/video)
+-------------------------------------┼-----------------------------------------------------------+
|  TẦNG 2: THỊ GIÁC MÁY TÍNH & NHẬN DIỆN CỬ CHỈ AI (WSL2 / ROS 2 HUMBLE)                         |
|  [ gesture_recognition_node ]                                                                   |
|       ├─ Video Stream Capture (HTTP/OpenCV, 30 FPS, Frame Flip H-Mirror)                        |
|       ├─ Auto-Brightness Booster (Bù sáng tự động khi môi trường thiếu sáng < 110 lum)          |
|       ├─ [ gesture_classifier ] Multi-Modal AI Perception (MediaPipe + Wave + Geometric)        |
|       ├─ Debounce & Hysteresis Filter (Khử nhiễu rung lắc, yêu cầu 5 frame liên tiếp)          |
|       └─ HUD Graphical Rendering (Vẽ khung xương bàn tay, thước đo độ tin cậy, trạng thái robot)|
|                                     │                                                           |
|                                     ▼ ROS 2 Topic: /detected_gesture (std_msgs/String @ 10 Hz)  |
+-------------------------------------┼-----------------------------------------------------------+
                                      │
+-------------------------------------┼-----------------------------------------------------------+
|  TẦNG 3: BỘ RA QUYẾT ĐỊNH & MÁY TRẠNG THÁI HỮU HẠN FSM (ROBOT BEHAVIOR SYSTEM)                  |
|  [ robot_behavior_node ]                                                                        |
|       ├─ Finite State Machine: IDLE ──► PENDING ──► EXECUTING ──► STOPPED / EMERGENCY           |
|       ├─ Quy trình xác nhận an toàn 2 bước: Chọn lệnh ──► Giơ Thumbs Up kích hoạt               |
|       ├─ Grace Cooldown Lockout (Khóa đệm 2.0s chống nhận nhầm khi người dùng hạ tay xuống)     |
|       ├─ Simulation Clock Synchronizer (Đồng bộ nhịp thời gian vật lý /clock từ Gazebo)         |
|       ├─ Watchdog Communication Safety (Ngắt an toàn nếu mất tín hiệu quá 15 giây)              |
|       └─ Feedback Status Publisher: /robot_behavior_status (Cập nhật ngược lên màn hình camera) |
|                                     │                                                           |
|                                     ▼ ROS 2 Topic: /cmd_vel & /diffdrive_controller/cmd_vel_unstamped
+-------------------------------------┼-----------------------------------------------------------+
                                      │ (ROS-GZ Parameter Bridge: ignition.msgs.Twist)
+-------------------------------------┼-----------------------------------------------------------+
|  TẦNG 4: MÔ PHỎNG VẬT LÝ 3D & TRUYỀN ĐỘNG ROBOT (GAZEBO IGNITION SIMULATOR)                    |
|  [ Gazebo Ignition Simulator (Depot World) ]                                                   |
|       ├─ TurtleBot 4 Standard (Differential Drive Mobile Base)                                 |
|       ├─ diffdrive_controller (ros2_control điều khiển 2 bánh xe chủ động)                      |
|       └─ Cửa sổ quan sát trực quan 3D Gazebo GUI (Real-time Kinematics Monitoring)              |
+-------------------------------------------------------------------------------------------------+
```﻿
---

## Bảng Ánh Xạ Cử Chỉ Tay & Hành Vi Robot (Gesture Mapping)

| STT | Cử chỉ (MediaPipe) | Tên & Đặc điểm nhận dạng | Hành vi Robot TurtleBot 4 | Thông số Động học | Quy trình kích hoạt |
|:---:|---|---|---|---|:---:|
| **1** | **`Wave`** | **Vẫy tay**<br>(Dao động cổ tay qua lại, bàn tay mở hướng lên) | Robot thực hiện hành vi **CHÀO LẠI (Wave Greeting)**: Tự động xoay trái - phải thân xe để đáp lễ. | $\omega = 0.50\text{ rad/s}$<br>Quay trái 1.0s, quay phải 2.0s, về giữa 1.0s.<br>Tổng thời gian: $4.0\text{s}$. | Cần giơ `Thumbs_Up` để xác nhận |
| **2** | **`Pointing_Up`** | **Giơ 1 ngón tay**<br>(Ngón trỏ duỗi thẳng đứng, 3 ngón giữa/nhẫn/út gập) | Robot thực hiện hành vi **TIẾN - LÙI 4 LẦN (Forward-Backward 4x)** dứt khoát, biên độ rõ ràng. | $v = \pm 0.35\text{ m/s}$<br>Tiến 3.5s ($+1.22\text{ m}$), dừng 0.5s,<br>Lùi 3.5s ($-1.22\text{ m}$), dừng 0.5s.<br>Lặp lại 4 chu kỳ. | Cần giơ `Thumbs_Up` để xác nhận |
| **3** | **`Victory`** | **Giơ 2 ngón tay**<br>(Ngón trỏ & giữa mở chữ V, ngón nhẫn & út gập) | Robot di chuyển theo **QUỸ ĐẠO TRÒN 360 ĐỘ (Circular Trajectory)** hoàn hảo khép kín. | $v = 0.35\text{ m/s}$<br>$\omega = 0.50\text{ rad/s}$ ($R = 0.70\text{ m}$)<br>Thời gian $T = \frac{2\pi}{\omega} = 12.57\text{s}$ sim time (Đúng 360°). | Cần giơ `Thumbs_Up` để xác nhận |
| **4** | **`Thumbs_Up`** | **Giơ ngón cái**<br>(Ngón cái hướng lên, 4 ngón còn lại cuộn chặt) | **CỬ CHỈ KÍCH HOẠT (CONFIRMATION):** Chuyển trạng thái từ `PENDING` sang `EXECUTING`, cho phép robot chạy. | Không phát vận tốc trực tiếp.<br>Kích hoạt timer FSM và đặt Grace Cooldown $2.0\text{s}$. | Lệnh kích hoạt thực thi (Tức thời) |
| **5** | **`Open_Palm`** | **Xòe bàn tay**<br>(Cả 5 ngón tay mở rộng, hướng về camera) | **DỪNG KHẨN CẤP (EMERGENCY STOP):** Ngắt tức thời toàn bộ vận tốc robot về 0 từ bất kỳ trạng thái nào. | $v = 0.0\text{ m/s}$, $\omega = 0.0\text{ rad/s}$<br>Thời gian phản ứng: $< 50\text{ ms}$ (Ưu tiên cao nhất để bảo đảm an toàn). | Kích hoạt tức thời |
| **6** | **`Closed_Fist`** | **Nắm tay**<br>(Cả 4 ngón cuộn chặt vào lòng bàn tay, ngón cái ép ngang) | **HỦY LỆNH & RESET (CANCEL):** Hủy bỏ hành vi đang chạy, dừng robot và đưa FSM về trạng thái `IDLE`. | $v = 0.0\text{ m/s}$, $\omega = 0.0\text{ rad/s}$<br>Reset toàn bộ chu kỳ tiến-lùi và quỹ đạo tròn. | Kích hoạt tức thời |

---

##  Hướng Dẫn Cài Đặt & Sử Dụng (Quick Start)

### 1. Yêu cầu Tiên quyết (Prerequisites)
- **Hệ điều hành:** Ubuntu 22.04 LTS (hoặc WSL2 trên Windows 10/11).
- **ROS 2:** Humble Hawksbill.
- **Mô phỏng:** Ignition Gazebo Fortress & TurtleBot 4 Simulation.
- **Python:** 3.10+ kèm các thư viện:
  ```bash
  pip install opencv-python mediapipe numpy
  ```

### 2. Biên dịch gói ROS 2 (Build Package)
```bash
# Di chuyển vào ROS 2 Workspace
cd ~/ros2_ws

# Biên dịch gói turtlebot4_gesture_control
colcon build --packages-select turtlebot4_gesture_control --symlink-install

# Cập nhật môi trường ROS 2
source install/setup.bash
```

---

### 3. Hướng dẫn Khởi chạy Hệ thống (Execution)

#### Cách 1: Chạy Tự Động Tất Cả Trong 1 Lệnh (Khuyên Dùng)

1. **Trên Windows (nếu dùng WSL2):**
   - Mở thư mục bài tập trong File Explorer: `\Turtlebot_4_cuchitay\`
   - Nhấp đúp chuột vào tệp **`run_camera_windows.bat`** để mở Camera Streamer (giữ cửa sổ này mở).
   *(Nếu bạn dùng Ubuntu cài trực tiếp trên máy thật, webcam `/dev/video0` sẽ được nhận tự động, bạn có thể bỏ qua bước này)*.

2. **Trong terminal Ubuntu (WSL2):**
   ```bash
   cd ~/ros2_ws/src/Turtlebot_4_cuchitay
   ./run_all.sh
   ```
   - Nhấn phím **`1`** (chế độ mặc định). Hệ thống sẽ tự động khởi động:
     - Gazebo Simulation Server & 3D GUI Window.
     - Node Nhận diện AI Vision & Giao diện HUD.
     - Node Điều khiển Robot Behavior FSM.

---

#### Cách 2: Khởi chạy Từng Phần Độc Lập

- **Terminal 1 - Mở mô phỏng Gazebo 3D:**
  ```bash
  cd ~/ros2_ws/src/Turtlebot_4_cuchitay
  ./run_simulation.sh
  ```
- **Terminal 2 - Mở AI Gesture Control:**
  ```bash
  cd ~/ros2_ws/src/Turtlebot_4_cuchitay
  ./run_gesture_control.sh
  ```

---

## Danh Mục ROS 2 Topics

| Tên Topic | Kiểu dữ liệu | Tần số | Mô tả chức năng |
|---|---|:---:|---|
| `/detected_gesture` | `std_msgs/String` | 10 Hz | Tên cử chỉ đã qua lọc nhiễu (`Pointing_Up`, `Victory`, `Thumbs_Up`, ...) |
| `/robot_behavior_status` | `std_msgs/String` | 10 Hz | Trạng thái hiện tại của Robot (`IDLE`, `PENDING`, `DANG CHAY...`) |
| `/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | Lệnh vận tốc chuẩn điều khiển TurtleBot 4 |
| `/diffdrive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | 20 Hz | Lệnh vận tốc trực tiếp cho bộ điều khiển 2 bánh xe |
| `/camera/gesture_annotated` | `sensor_msgs/Image` | 30 Hz | Hình ảnh camera kèm HUD và các khớp tay để hiển thị trên RViz2 |
| `/clock` | `rosgraph_msgs/Clock` | 100 Hz | Xung nhịp thời gian mô phỏng vật lý đồng bộ từ Gazebo Ignition |

---


