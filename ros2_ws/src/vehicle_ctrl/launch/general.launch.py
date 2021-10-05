import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    rviz_pkg_dir = os.path.join(get_package_share_directory('vehicle_ctrl'), 'rviz', 'general.rviz')
    print(rviz_pkg_dir)

    description_pkg_dir = os.path.join(get_package_share_directory('vehicle_description'), 'launch')
    print(description_pkg_dir)

    return LaunchDescription([
        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz2',
            arguments = ['-d', rviz_pkg_dir],
            # output = 'screen'
        ),

        Node(
            package = 'image_transport',
            executable = 'republish',
            arguments = ['compressed', '--ros-args', '--remap', 'in/compressed:=vehicle/camera/compressed'],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_pkg_dir, '/publish.launch.py']),
        ),

        ExecuteProcess(
            cmd = ['gnome-terminal','--', 'bash', '-c', 'ros2 run vehicle_ctrl keyboard'],
        ),
    ])