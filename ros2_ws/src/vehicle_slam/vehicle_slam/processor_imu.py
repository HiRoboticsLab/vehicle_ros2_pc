import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu as MsgImu
from std_msgs.msg import Float64MultiArray
import PyKDL
import math

class Talker(Node):
    def __init__(self):
        super().__init__('processor_imu')
        self.listener_imu = self.create_subscription(Float64MultiArray, '/vehicle/imu_ypr', self.on_receive_msg, 10)
        self.talker_imu = self.create_publisher(MsgImu, '/processor/imu', 10)
        # 记录提交时间
        self.last_time = self.get_clock().now()
        self.last_angle = 0
        
    def on_receive_msg(self, msg):
        # self.get_logger().info('receive ypr --> data : "%s"' % msg)
        try:
            current_time = self.get_clock().now()

            angle = msg.data[0] * (-1.0)
            delta_angle = angle - self.last_angle
            delta_radian = delta_angle / 360 * 2 * math.pi
            self.last_angle = angle

            # 防止跳变
            if abs(delta_angle) < 3:
                # self.get_logger().error('！！！！！')

                quaternion = PyKDL.Rotation.RPY(
                    msg.data[2] / 180 * math.pi, \
                    msg.data[1] / 180 * math.pi, \
                    msg.data[0] / 180 * math.pi  \
                ).GetQuaternion()

                imu = MsgImu()

                imu.header.stamp = current_time.to_msg()
                imu.header.frame_id = "imu_link"

                imu.orientation.x =  quaternion[0] * 1.0
                imu.orientation.y =  quaternion[1] * 1.0
                imu.orientation.z =  quaternion[2] * 1.0
                imu.orientation.w =  quaternion[3] * 1.0

                imu.linear_acceleration.z = 0.001
                imu.angular_velocity.z = delta_radian * 20 * 1.0

                self.talker_imu.publish(imu)
        except Exception as e:
            self.get_logger().error('%s' % e)


def main(args = None):
    rclpy.init(args = args)

    talker = Talker()

    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

