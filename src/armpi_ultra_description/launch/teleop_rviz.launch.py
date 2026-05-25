"""
teleop_rviz.launch.py

Launches robot_state_publisher + RViz only — no joint_state_publisher_gui.
Use this alongside teleop_ik_v2.py, which publishes /joint_states itself.

  Terminal 1:  ros2 launch armpi_ultra_description teleop_rviz.launch.py
  Terminal 2:  python3 scripts/teleop_ik_v2.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg       = get_package_share_directory('armpi_ultra_description')
    urdf_path = os.path.join(pkg, 'urdf', 'armpi_ultra.urdf')
    rviz_path = os.path.join(pkg, 'rviz', 'arm.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
        ),
    ])
