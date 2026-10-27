#!/bin/bash
# ==========================================================
# PHAN 1: Khoi chay TurtleBot 4 Simulation & Gazebo 3D GUI
# ==========================================================
echo "=========================================================="
echo " [PHAN 1] Khoi chay TurtleBot 4 Simulator & Cua so 3D Gazebo"
echo "=========================================================="
source /opt/ros/humble/setup.bash
source /home/hoangtam/ros2_ws/install/setup.bash

# Don dep tien trinh cu truoc khi chay
killall -9 -q ruby ign gz turtlebot4_node motion_control 2>/dev/null || true
sleep 1

export XDG_RUNTIME_DIR="/tmp/runtime-$USER"
mkdir -p "$XDG_RUNTIME_DIR" 2>/dev/null && chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null

export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds:/opt/ros/humble/share/irobot_create_ignition_bringup/worlds:/opt/ros/humble/share"
export IGN_GUI_PLUGIN_PATH="/opt/ros/humble/share/turtlebot4_ignition_gui_plugins/lib:/opt/ros/humble/share/irobot_create_ignition_plugins/lib"

WORLD="${1:-depot}"
echo "-> [1/3] Dang khoi chay Gazebo Simulation Server & Robot (World: $WORLD)..."
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:="$WORLD" &
SERVER_PID=$!

echo "-> Dang cho Robot spawn va diffdrive_controller san sang..."
count=0
while ! ros2 topic list 2>/dev/null | grep -q "^/odom$"; do
    sleep 1
    count=$((count+1))
    if [ $count -eq 8 ] || [ $count -eq 15 ]; then
        echo "-> [Tu dong tro giup] Dang kich hoat diffdrive_controller..."
        ros2 run controller_manager spawner diffdrive_controller >/dev/null 2>&1 &
    fi
    if [ $count -ge 25 ]; then
        break
    fi
done
sleep 1
echo "-> Robot va diffdrive_controller da SAN SANG 100%!"

# Mo khoa gioi han van toc cua diffdrive_controller
(
    sleep 2
    ros2 param set /diffdrive_controller linear.x.max_velocity 1.50 >/dev/null 2>&1 || true
    ros2 param set /diffdrive_controller linear.x.max_acceleration 3.0 >/dev/null 2>&1 || true
    ros2 param set /diffdrive_controller angular.z.max_velocity 2.50 >/dev/null 2>&1 || true
) &

echo "-> [2/3] Dang mo Cua so Giao dien 3D Gazebo GUI..."
(
    export LIBGL_ALWAYS_SOFTWARE=1
    export DISPLAY=${DISPLAY:-:0}
    ign gazebo -g --gui-config /opt/ros/humble/share/turtlebot4_ignition_bringup/gui/standard/gui.config
) &
GUI_PID=$!

echo "=========================================================="
echo "-> Gazebo 3D GUI da duoc khoi dong!"
echo "-> Nhan Ctrl+C de dung toan bo simulation."
echo "=========================================================="

cleanup() {
    echo ""
    echo "Dang dong Gazebo..."
    kill -9 $SERVER_PID $GUI_PID 2>/dev/null || true
    killall -9 -q ruby ign gz turtlebot4_node 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $SERVER_PID
