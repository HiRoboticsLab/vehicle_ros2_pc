import json
from rclpy.node import Node
from std_msgs.msg import String


class Light(Node):


    def __init__(self):
        super().__init__('mission_light')
        self.pub_light = self.create_publisher(String, '/vehicle/cmd_light', 10)


    def send(self):
        light_msg = String()
        light_msg.data = 'head'
        self.pub_light.publish(light_msg)
        light_msg.data = 'both'
        self.pub_light.publish(light_msg)
