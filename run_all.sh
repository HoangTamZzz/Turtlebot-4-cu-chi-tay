#!/bin/bash
# ==========================================================
# BÀI TẬP 6 – GIAO TIẾP PHI NGÔN NGỮ VÀ ĐIỀU KHIỂN TURTLEBOT 4
# ==========================================================

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source /home/hoangtam/ros2_ws/install/setup.bash

export DISPLAY=${DISPLAY:-:0}
export LIBGL_ALWAYS_SOFTWARE=1

echo "=========================================================="
echo "  BÀI TẬP 6 - ĐIỀU KHIỂN TURTLEBOT 4 BẰNG CỬ CHỈ TAY"
echo "  (Hệ thống CHỈ nhận lệnh từ cử chỉ tay qua Webcam)"
echo "=========================================================="
echo "  1) Chạy Toàn bộ (Gazebo Server + 3D GUI + Nhận diện Cử chỉ)"
echo "  2) Chạy Mô phỏng Gazebo (Server + 3D GUI)"
echo "  3) Chạy Node Nhận diện Cử chỉ & Điều khiển Robot"
echo "  4) Dọn dẹp toàn bộ tiến trình"
echo "=========================================================="
read -p "Chọn chế độ (1-4, mặc định 1): " choice
choice=${choice:-1}

cleanup() {
    echo ""
    echo "Dang dung toan bo cac tien trinh..."
    kill $(jobs -p) 2>/dev/null || true
    killall -9 -q ruby ign gz turtlebot4_node motion_control 2>/dev/null || true
    pkill -9 -f "gesture_recognition_node" 2>/dev/null || true
    pkill -9 -f "robot_behavior_node" 2>/dev/null || true
    echo "Da don dep sach se!"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

case "$choice" in
    1)
        echo "-> [1/2] Đang khởi động Gazebo Simulation & 3D GUI..."
        bash "$DIR/run_simulation.sh" &
        sleep 8

        echo "-> [2/2] Đang khởi động AI Gesture Recognition & Robot Control..."
        bash "$DIR/run_gesture_control.sh"
        ;;
    2)
        bash "$DIR/run_simulation.sh"
        ;;
    3)
        bash "$DIR/run_gesture_control.sh"
        ;;
    4)
        cleanup
        ;;
    *)
        echo "Lựa chọn không hợp lệ."
        ;;
esac
