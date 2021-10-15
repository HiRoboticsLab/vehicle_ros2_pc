import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    

    mission_pkg_dir = os.path.join(get_package_share_directory('vehicle_mission'), 'launch')
    print(mission_pkg_dir)

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([mission_pkg_dir, '/build_map.launch.py']),
        ),

        Node(
            package = 'vehicle_mission',
            executable = 'gogogo',
            name = 'mission_go',
        ),
    ]) 