import os
import json
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from vehicle_kinematics.kinematics import Kinematics
from ament_index_python.packages import get_package_share_directory


class Controller(Node):


    def __init__(self):
        super().__init__('mission_controller')
        self.pub_wheel = self.create_publisher(Int32MultiArray, '/vehicle/cmd_wheel', 10)

        # 加载运动学节点
        config_file_path = os.path.join(get_package_share_directory('vehicle_kinematics'), 'config', 'vehicle.json')
        file = open(config_file_path, 'rb')
        config_json = json.load(file)
        self.kinematics = Kinematics(config_json['wheel_distance'], config_json['wheel_diameter'], config_json['wheel_laps_code'])


    def send(self, linear, angular):
        # self.get_logger().info('I heard: "%s"' % msg)
        linear = [linear, 0, 0]
        angular = [0, 0, angular]
        left, right = self.kinematics.forward(linear, angular)
        
        wheel_msg = Int32MultiArray()
        wheel_msg.data = [int(left), int(right)]
        self.pub_wheel.publish(wheel_msg)
