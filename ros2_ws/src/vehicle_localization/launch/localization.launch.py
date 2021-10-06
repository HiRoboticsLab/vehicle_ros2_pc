import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    rviz_pkg_dir = os.path.join(get_package_share_directory('vehicle_localization'), 'rviz', 'localization.rviz')
    print(rviz_pkg_dir)

    ctrl_pkg_dir = os.path.join(get_package_share_directory('vehicle_ctrl'), 'launch')
    print(ctrl_pkg_dir)

    description_pkg_dir = os.path.join(get_package_share_directory('vehicle_description'), 'launch')
    print(description_pkg_dir)

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_pkg_dir, '/publish.launch.py']),
        ),

        # Node(
        #     package='vehicle_slam',
        #     executable='processor_imu',
        #     # output='screen',
        # ),

        Node(
            package = 'cartographer_ros',
            executable='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', get_package_share_directory('vehicle_localization') + '/config',
                '-configuration_basename', 'localization.lua',
                '-load_state_filename', get_package_share_directory('vehicle_localization') + '/map' + '/map.pbstream',
            ],
            remappings=[
                ('/scan', '/vehicle/scan'),
                ('/imu', '/processor/imu'),
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            arguments=[
                '-resolution', '0.05'
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ctrl_pkg_dir, '/base.launch.py']),
        ),

        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz2',
            arguments = ['-d', rviz_pkg_dir],
            # output = 'screen'
        ),
    ]) 