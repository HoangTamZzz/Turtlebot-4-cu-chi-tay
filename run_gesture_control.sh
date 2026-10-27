#!/bin/bash
echo "=========================================================="
echo " [PHAN 2 & 3] Khoi chay AI Nhan Dien Cu Chi Tay & Robot Control"
echo "=========================================================="
source /opt/ros/humble/setup.bash
source /home/hoangtam/ros2_ws/install/setup.bash

# Ensure Display is exported for WSLg GUI
export DISPLAY=${DISPLAY:-:0}

echo "-> Dang khoi dong cac ROS 2 Node..."
echo "-> Cua so webcam MediaPipe va Robot Behavior dang bat..."
ros2 launch turtlebot4_gesture_control gesture_control.launch.py show_gui:=True
