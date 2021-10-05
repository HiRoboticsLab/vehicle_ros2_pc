import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory('vehicle_description'), 'urdf', 'out.urdf')
    print(urdf_file)

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            name = 'robot_state_publisher',
            output = 'screen',
            parameters = [
                {'robot_description': robot_desc},
                # {'use_sim_time': False}
            ]
        ),
    ])