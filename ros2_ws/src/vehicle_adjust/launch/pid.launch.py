import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    ctrl_pkg_dir = os.path.join(get_package_share_directory('vehicle_ctrl'), 'launch')
    print(ctrl_pkg_dir)

    return LaunchDescription([

        Node(
            package = 'vehicle_adjust',
            executable = 'pid',
            name = 'vehicle_adjust',
        ),

        Node(
            package = 'rqt_reconfigure',
            executable = 'rqt_reconfigure',
            name = 'rqt_reconfigure',
        ),

        Node(
            package = 'rqt_plot',
            executable = 'rqt_plot',
            name = 'rqt_plot',
            arguments = ['/vehicle/wheel/data[0]', '/vehicle/wheel/data[1]'],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ctrl_pkg_dir, '/base.launch.py']),
        ),
    ]) 