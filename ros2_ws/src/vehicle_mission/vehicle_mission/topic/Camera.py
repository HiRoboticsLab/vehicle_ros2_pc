import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np


class Camera(Node):

    # 回调声明
    callback_image = None


    def __init__(self, callback_image):
        super().__init__('mission_camera')
        self.listener_image = self.create_subscription(CompressedImage, '/vehicle/camera/compressed', self.on_receive_image, 10)
        self.callback_image = callback_image


    def on_receive_image(self, msg):
        try:
            # self.get_logger().info('receive image --> data : "%s"' % msg.data)
            img_array = np.asarray(msg.data, dtype="uint8")
            cv_image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            # 回调显示
            self.callback_image(cv_image)

            # 必须加上，否则不显示
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error('%s' % e)

