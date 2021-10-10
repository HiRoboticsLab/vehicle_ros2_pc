import json
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import math
# 四元数转欧拉角
import PyKDL


# 使用localization项目确定坐标系
class Transform(Node):
    
    x = 0
    y = 0

    rx = 0
    ry = 0
    rz = 0
    rw = 0


    def __init__(self):
        super().__init__('mission_tf')
        self.create_subscription(TFMessage, '/tf', self.on_receive, 10)


    def on_receive(self, msg):
        # self.get_logger().info('%s' % msg)
        self.x = msg.transforms[0].transform.translation.x
        self.y = msg.transforms[0].transform.translation.y

        self.rx = msg.transforms[0].transform.rotation.x
        self.ry = msg.transforms[0].transform.rotation.y
        self.rz = msg.transforms[0].transform.rotation.z
        self.rw = msg.transforms[0].transform.rotation.w


    def get_x(self):
        return self.x


    def get_y(self):
        return self.y


    def get_angle(self):
        rpy = PyKDL.Rotation.Quaternion(self.rx, self.ry, self.rz, self.rw).GetRPY()
        angle = math.degrees(rpy[2])
        # print(rpy[2])
        # print(angle)
        return angle


        