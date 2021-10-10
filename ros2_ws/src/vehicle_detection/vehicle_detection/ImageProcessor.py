import cv2
import numpy as np


class ImageProcessor():
    # 形态梯度参数
    kernel = 0
    threshold_low = 0
    threshold_high = 0
    mask_height = 0

    # HSV参数
    threshold_hsv_lower_h = 0
    threshold_hsv_lower_s = 0
    threshold_hsv_lower_v = 0
    threshold_hsv_upper_h = 0
    threshold_hsv_upper_s = 0
    threshold_hsv_upper_v = 0


    def morph_gradient(image):
        try:
            # 形态
            kernel = np.ones((ImageProcessor.kernel, ImageProcessor.kernel), np.uint8)
            gradient = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, kernel)
            # 灰度
            gray = cv2.cvtColor(gradient, cv2.COLOR_BGR2GRAY)
            # 背景图
            bg = np.copy(gray)
            bg[::] = 0
            # 图片非操作
            result = cv2.bitwise_not(gray, bg)

            ret, thresh = cv2.threshold(result, ImageProcessor.threshold_low, ImageProcessor.threshold_high, cv2.THRESH_BINARY)
            return thresh
        except Exception as e:
            print(e)


    def hsv(image):
        lower = np.array([ImageProcessor.threshold_hsv_lower_h, ImageProcessor.threshold_hsv_lower_s, ImageProcessor.threshold_hsv_lower_v])
        upper = np.array([ImageProcessor.threshold_hsv_upper_h, ImageProcessor.threshold_hsv_upper_s, ImageProcessor.threshold_hsv_upper_v])
        # gauss = cv2.GaussianBlur(image, (9, 9), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        result = cv2.inRange(hsv, lower, upper)
        return result


    def find_center(image, thresh):
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

            return center_x, center_y, w, h, image
        except Exception as e:
            print(e)

