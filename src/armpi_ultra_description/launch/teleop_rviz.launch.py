"""
teleop_rviz.launch.py

Launches camera driver + robot_state_publisher + RViz.
Use this alongside teleop_ik_v2.py, which publishes /joint_states itself.

  Terminal 1:  ros2 launch armpi_ultra_description teleop_rviz.launch.py
  Terminal 2:  python3 scripts/teleop_ik_v2.py

Optional argument:
  rviz_config:=arm_3dviz.rviz   (default: arm_3dviz.rviz)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg       = get_package_share_directory('armpi_ultra_description')
    urdf_path = os.path.join(pkg, 'urdf', 'armpi_ultra.urdf')

    rviz_config = LaunchConfiguration('rviz_config', default='arm_3dviz.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='deptrum-ros-driver-aurora930',
            executable='aurora930_node',
            namespace='aurora',
            parameters=[{
                'rgb_enable':        True,
                'depth_enable':      True,
                'ir_enable':         False,
                'point_cloud_enable': False,
                'rgb_fps':           12,
                'ir_fps':            12,
                'align_mode':        True,
                'depth_correction':  True,
            }],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3',
                 os.path.expanduser('~/ros2arm_ws/scripts/crosshair_overlay.py')],
            output='screen',
        ),
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
            arguments=['-d', [os.path.join(pkg, 'rviz', ''), rviz_config]],
        ),
    ])
