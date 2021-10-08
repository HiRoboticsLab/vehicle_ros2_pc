import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
from rcl_interfaces.msg import SetParametersResult


class ImageListener(Node):

    # 形态梯度参数
    kernel = 0
    threshold_low = 0
    threshold_high = 0
    mask_height = 0
    callback_center = None

    # HSV参数
    threshold_hsv_lower_h = 0
    threshold_hsv_lower_s = 0
    threshold_hsv_lower_v = 0
    threshold_hsv_upper_h = 0
    threshold_hsv_upper_s = 0
    threshold_hsv_upper_v = 0


    def __init__(self):
        super().__init__('camera_processor_morph')
        self.listener_image = self.create_subscription(CompressedImage, '/vehicle/camera/compressed', self.on_receive_image, 10)


    def on_receive_image(self, msg):
        try:
            # self.get_logger().info('receive image --> data : "%s"' % msg.data)
            img_array = np.asarray(msg.data, dtype="uint8")
            cv_image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            # hsv处理
            hsv = self.hsv(cv_image)
            hsv[0: self.mask_height] = 0
            cv2.imshow('image_hsv', hsv)
            
            # 形态梯度处理
            morph = self.morph_gradient(cv_image)
            morph[0: self.mask_height] = 0
            cv2.imshow('image_morph', morph)

            # 结合
            both = cv2.bitwise_and(morph, hsv)
            cv2.imshow('both', both)

            # TODO 不想用hsv，可直接互换注释
            # center = self.find_center(cv_image, morph)
            center = self.find_center(cv_image, both)

            cv2.imshow('image_center', center)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error('%s' % e)


    def morph_gradient(self, image):
        try:
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
        except Exception as e:
            self.get_logger().error('%s' % e)


    def hsv(self, image):
        lower = np.array([self.threshold_hsv_lower_h, self.threshold_hsv_lower_s, self.threshold_hsv_lower_v])
        upper = np.array([self.threshold_hsv_upper_h, self.threshold_hsv_upper_s, self.threshold_hsv_upper_v])
        # gauss = cv2.GaussianBlur(image, (9, 9), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        result = cv2.inRange(hsv, lower, upper)
        return result


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

            if self.callback_center:
                self.callback_center(center_x, center_y, w, h, image)

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

            if(param.name == 'threshold_hsv_lower_h'):
                self.threshold_hsv_lower_h = param.value
            if(param.name == 'threshold_hsv_lower_s'):
                self.threshold_hsv_lower_s = param.value
            if(param.name == 'threshold_hsv_lower_v'):
                self.threshold_hsv_lower_v = param.value
            if(param.name == 'threshold_hsv_upper_h'):
                self.threshold_hsv_upper_h = param.value
            if(param.name == 'threshold_hsv_upper_s'):
                self.threshold_hsv_upper_s = param.value
            if(param.name == 'threshold_hsv_upper_v'):
                self.threshold_hsv_upper_v = param.value

        return SetParametersResult(successful=True)