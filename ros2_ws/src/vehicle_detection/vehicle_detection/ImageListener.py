import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from .ImageProcessor import ImageProcessor


class ImageListener(Node):

    # 回调声明
    callback_image = None


    def __init__(self, callback_image):
        super().__init__('camera_processor_morph')
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


    # 此处为ros2节点参数回调
    def callback(self, params):
        for param in params:
            self.get_logger().info(param.name)
            self.get_logger().info(str(param.value))
            self.get_logger().info(str(type(param.value)))

            if(param.name == 'kernel'):
                ImageProcessor.kernel = param.value
            if(param.name == 'threshold_low'):
                ImageProcessor.threshold_low = param.value
            if(param.name == 'threshold_high'):
                ImageProcessor.threshold_high = param.value

            if(param.name == 'threshold_hsv_lower_h'):
                ImageProcessor.threshold_hsv_lower_h = param.value
            if(param.name == 'threshold_hsv_lower_s'):
                ImageProcessor.threshold_hsv_lower_s = param.value
            if(param.name == 'threshold_hsv_lower_v'):
                ImageProcessor.threshold_hsv_lower_v = param.value
            if(param.name == 'threshold_hsv_upper_h'):
                ImageProcessor.threshold_hsv_upper_h = param.value
            if(param.name == 'threshold_hsv_upper_s'):
                ImageProcessor.threshold_hsv_upper_s = param.value
            if(param.name == 'threshold_hsv_upper_v'):
                ImageProcessor.threshold_hsv_upper_v = param.value

        return SetParametersResult(successful = True)