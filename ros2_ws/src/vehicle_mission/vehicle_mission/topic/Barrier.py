import json
from rclpy.node import Node
from std_msgs.msg import String


class Barrier(Node):


    def __init__(self):
        super().__init__('mission_barrier')
        self.pub_barrier = self.create_publisher(String, 'vehicle/cmd_esp', 10)


    def open(self, id = "null"):
        data = {}
        data['cmd'] = 'open'
        data['id'] = id
        barrier_msg = String()
        barrier_msg.data = str(data)
        self.pub_barrier.publish(barrier_msg)


    def close(self, id = "null"):
        data = {}
        data['cmd'] = 'close'
        data['id'] = id
        barrier_msg = String()
        barrier_msg.data = str(data)
        self.pub_barrier.publish(barrier_msg)