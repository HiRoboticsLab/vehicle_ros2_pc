import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
import io
from PIL import Image
import numpy as np
from rcl_interfaces.msg import SetParametersResult


class ImageListener(Node):

    kernel = 0
    threshold_low = 0
    threshold_high = 0

    def __init__(self):
        super().__init__('processor_camera_morph')
        self.listener_image = self.create_subscription(CompressedImage, '/vehicle/camera/compressed', self.on_receive_image, 10)


    def on_receive_image(self, msg):
        try:
            # self.get_logger().info('receive image --> data : "%s"' % msg.data)
            img_array = np.asarray(msg.data, dtype="uint8")
            cv_image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            # TODO
            morph = self.morph_gradient(cv_image)
            center = self.find_center(cv_image, morph)

            cv2.imshow('image_morph', morph)
            cv2.imshow('image_center', center)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error('%s' % e)


    def morph_gradient(self, image):
        # 形态
        kernel = np.ones((self.kernel, self.kernel), np.uint8)
        gradient = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, kernel)
        # 灰度
        gray = cv2.cvtColor(gradient, cv2.COLOR_BGR2GRAY)
        # 背景图
        bg = np.copy(gray)
        bg[::] = 0
        # 图片非操作
        result = cv2.bitwise_not(gray, bg)

        ret, thresh = cv2.threshold(result, self.threshold_low, self.threshold_high, cv2.THRESH_BINARY)
        return thresh


    def find_center(self, image, thresh):
        try:
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            contours.sort(key = cv2.contourArea, reverse=True)
            
            (x, y, w, h) = cv2.boundingRect(contours[0])
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2, cv2.LINE_AA)
            # 中点
            M = cv2.moments(contours[0])
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            cv2.circle(image, (center_x, center_y), 1, (0, 0, 255), -1)
            return image
        except Exception as e:
            self.get_logger().error('%s' % e)



    def callback(self, params):
        for param in params:
            self.get_logger().info(param.name)
            self.get_logger().info(str(param.value))
            self.get_logger().info(str(type(param.value)))

            if(param.name == 'kernel'):
                self.kernel = param.value
            if(param.name == 'threshold_low'):
                self.threshold_low = param.value
            if(param.name == 'threshold_high'):
                self.threshold_high = param.value

        return SetParametersResult(successful=True)



def main(args = None):
    try:
        rclpy.init(args = args)

        image = ImageListener()

        image.declare_parameter('kernel', 10)
        image.declare_parameter('threshold_low', 200)
        image.declare_parameter('threshold_high', 255)

        (kernel, threshold_low, threshold_high) = image.get_parameters(
            ['kernel', 'threshold_low', 'threshold_high']
        )

        image.kernel = kernel.value
        image.threshold_low = threshold_low.value
        image.threshold_high = threshold_high.value

        image.add_on_set_parameters_callback(image.callback)
        
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