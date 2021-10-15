import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    rviz_pkg_dir = os.path.join(get_package_share_directory('vehicle_mission'), 'rviz', 'map.rviz')
    print(rviz_pkg_dir)

    slam_pkg_dir = os.path.join(get_package_share_directory('vehicle_mission'), 'launch')
    print(slam_pkg_dir)

    # ctrl_pkg_dir = os.path.join(get_package_share_directory('vehicle_ctrl'), 'launch')
    # print(ctrl_pkg_dir)

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_pkg_dir, '/cartographer.launch.py']),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ctrl_pkg_dir, '/base.launch.py']),
        # ),

        # Node(
        #     package = 'image_transport',
        #     executable = 'republish',
        #     arguments = ['compressed', '--ros-args', '--remap', 'in/compressed:=vehicle/camera/compressed'],
        # ),

        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz2',
            arguments = ['-d', rviz_pkg_dir],
            # output = 'screen'
        ),
    ]) 