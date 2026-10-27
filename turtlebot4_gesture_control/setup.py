import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot4_gesture_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoangtam',
    maintainer_email='hoangtam@todo.todo',
    description='ROS 2 Package for TurtleBot 4 Hand Gesture Recognition and Non-verbal Interaction',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognition_node = turtlebot4_gesture_control.gesture_recognition_node:main',
            'robot_behavior_node = turtlebot4_gesture_control.robot_behavior_node:main',
        ],
    },
)
