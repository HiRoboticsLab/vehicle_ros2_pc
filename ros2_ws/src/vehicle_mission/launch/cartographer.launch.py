import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    description_pkg_dir = os.path.join(get_package_share_directory('vehicle_description'), 'launch')
    print(description_pkg_dir)

    return LaunchDescription([

        # Node(
        #     package='vehicle_slam',
        #     executable='processor_imu',
        #     # output='screen',
        # ),

        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package = 'cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', get_package_share_directory('vehicle_slam') + '/config',
                '-configuration_basename', 'cartographer.lua'
            ],
            remappings=[
                ('/scan', '/vehicle/scan'),
                ('/imu', '/processor/imu'),
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.02', '-publish_period_sec', '1.0']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_pkg_dir, '/publish.launch.py']),
        ),
    ]) 