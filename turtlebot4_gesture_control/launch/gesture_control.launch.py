import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot4_gesture_control')
    default_model_path = os.path.join(pkg_share, 'models', 'gesture_recognizer.task')

    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Device ID of webcam (e.g. 0, 1)'
    )

    video_source_arg = DeclareLaunchArgument(
        'video_source',
        default_value='',
        description='Optional video file path or video stream URL'
    )

    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='True',
        description='Whether to display OpenCV GUI window'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock from /clock'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.35',
        description='Base linear speed for robot movements (m/s)'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.50',
        description='Base angular speed for robot rotations (rad/s)'
    )

    gesture_node = Node(
        package='turtlebot4_gesture_control',
        executable='gesture_recognition_node',
        name='gesture_recognition_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'video_source': LaunchConfiguration('video_source'),
            'show_gui': LaunchConfiguration('show_gui'),
            'confidence_threshold': 0.55,
            'debounce_frames': 5,
            'model_path': default_model_path
        }]
    )

    behavior_node = Node(
        package='turtlebot4_gesture_control',
        executable='robot_behavior_node',
        name='robot_behavior_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'watchdog_timeout': 15.0
        }]
    )

    return LaunchDescription([
        camera_id_arg,
        video_source_arg,
        show_gui_arg,
        use_sim_time_arg,
        linear_speed_arg,
        angular_speed_arg,
        gesture_node,
        behavior_node
    ])
