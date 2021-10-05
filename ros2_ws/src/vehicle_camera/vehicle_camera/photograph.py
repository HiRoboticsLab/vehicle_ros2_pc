import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
import io
from PIL import Image
import numpy as np
import uuid
import os
from ament_index_python.packages import get_package_share_directory
# GUI库
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton


class ImageListener(Node):

    needSaveImage = False

    def __init__(self):
        super().__init__('processor_camera')
        self.listener_image = self.create_subscription(CompressedImage, '/vehicle/camera/compressed', self.on_receive_image, 10)


    def on_receive_image(self, msg):
        try:
            # self.get_logger().info('receive image --> data : "%s"' % msg.data)
            img_array = np.asarray(msg.data, dtype="uint8")
            cv_image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            cv2.imshow('image', cv_image)
            if self.needSaveImage:
                self.saveImage(cv_image)
                self.needSaveImage = False
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error('%s' % e)


    def saveImage(self, image):
        self.get_logger().info('saveImage！！！')
        filename = str(uuid.uuid1()) + '.jpg'
        directory = get_package_share_directory('vehicle_camera').replace("/install/vehicle_camera/share/vehicle_camera", "/images")
        # self.get_logger().info(directory)
        if not os.path.exists(directory):
            os.makedirs(directory)
        image_path = os.path.join(directory, filename)
        cv2.imwrite(image_path, image)
        self.get_logger().info(image_path)

    def save(self):
        self.needSaveImage = True


class App(QWidget):
    def __init__(self, callback):
        super().__init__()
        self.title = '拍照'
        self.left = 0
        self.top = 0
        self.width = 320
        self.height = 240

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        
        button = QPushButton('拍照', self)
        button.move(120, 100)
        button.clicked.connect(callback)
        
        self.show()


def main(args = None):
    try:
        rclpy.init(args = args)
        image = ImageListener()

        executor = SingleThreadedExecutor()
        executor.add_node(image)

        app = QApplication(sys.argv)
        exe = App(image.save)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            image.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
