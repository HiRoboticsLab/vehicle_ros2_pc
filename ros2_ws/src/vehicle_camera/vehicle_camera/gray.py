import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
import io
from PIL import Image
import numpy as np


class ImageListener(Node):
    def __init__(self):
        super().__init__('processor_camera')
        self.listener_image = self.create_subscription(CompressedImage, '/vehicle/camera/compressed', self.on_receive_image, 10)

        
    def on_receive_image(self, msg):
        try:
            # self.get_logger().info('receive image --> data : "%s"' % msg.data)
            img_array = np.asarray(msg.data, dtype="uint8")
            cv_image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            # TODO
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            cv2.imshow('image_gray', gray)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error('%s' % e)


def main(args = None):
    try:
        rclpy.init(args = args)
        image = ImageListener()
        executor = SingleThreadedExecutor()
        executor.add_node(image)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            image.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()