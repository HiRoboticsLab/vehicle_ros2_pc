import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    

    return LaunchDescription([

        ExecuteProcess(
            cmd = ['gnome-terminal','--', 'bash', '-c', 'ros2 run vehicle_ctrl keyboard'],
        ),
    ])