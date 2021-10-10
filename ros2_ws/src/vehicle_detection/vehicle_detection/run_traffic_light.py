import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from .ImageListener import ImageListener
from .ImageProcessor import ImageProcessor
import requests


# 使用前，电脑端需要执行 “pip3 install requests”
def callback_image(image):
    # 显示图片
    cv2.imshow('camera', image)
    try:
        if not (image is None):
            # result_img = np.copy(image)
            response = requests.post('http://127.0.0.1:24401/', params={'threshold': 0.1},
                                     data=bytes(cv2.imencode('.jpg', image)[1])).json()
            # print(response)
            if len(response['results']) != 0:
                # print(response['results'])
                max_confidence, max_index = 0, 0
                for index, result in enumerate(response['results']):
                    # print(result)
                    confidence = result['confidence']
                    if confidence > max_confidence:
                        max_confidence = confidence
                        max_index = index
                result = response['results'][max_index]
                confidence = result['confidence']
                label = result['label']
                x = result['location']['left']
                y = result['location']['top']
                w = result['location']['width']
                h = result['location']['height']
                # 输入参数为图像、左上角坐标、右下角坐标、颜色(B,G,R)数组、粗细
                image = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 1)
                # 输入参数为图像、文本、位置、字体、大小、颜色(B,G,R)数组、粗细
                image = cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # print(label, confidence, x, y, w, h)
                cv2.imshow('easydl', image)
    except Exception as e:
        print(e)


def main(args = None):
    try:
        rclpy.init(args = args)

        image = ImageListener(callback_image)

        # 形态梯度参数配置
        image.declare_parameter('kernel', 10)
        image.declare_parameter('threshold_low', 240)
        image.declare_parameter('threshold_high', 255)

        (kernel, threshold_low, threshold_high) = image.get_parameters(
            ['kernel', 'threshold_low', 'threshold_high']
        )

        ImageProcessor.kernel = kernel.value
        ImageProcessor.threshold_low = threshold_low.value
        ImageProcessor.threshold_high = threshold_high.value

        # 遮罩层高度
        image.mask_height = 240

        # HSV参数配置
        image.declare_parameter('threshold_hsv_lower_h', 100)
        image.declare_parameter('threshold_hsv_lower_s', 0)
        image.declare_parameter('threshold_hsv_lower_v', 80)
        image.declare_parameter('threshold_hsv_upper_h', 255)
        image.declare_parameter('threshold_hsv_upper_s', 255)
        image.declare_parameter('threshold_hsv_upper_v', 150)

        (threshold_hsv_lower_h, threshold_hsv_lower_s, threshold_hsv_lower_v, threshold_hsv_upper_h, threshold_hsv_upper_s, threshold_hsv_upper_v) = image.get_parameters(
            ['threshold_hsv_lower_h', 'threshold_hsv_lower_s', 'threshold_hsv_lower_v', 'threshold_hsv_upper_h', 'threshold_hsv_upper_s', 'threshold_hsv_upper_v']
        )

        ImageProcessor.threshold_hsv_lower_h = threshold_hsv_lower_h.value
        ImageProcessor.threshold_hsv_lower_s = threshold_hsv_lower_s.value
        ImageProcessor.threshold_hsv_lower_v = threshold_hsv_lower_v.value
        ImageProcessor.threshold_hsv_upper_h = threshold_hsv_upper_h.value
        ImageProcessor.threshold_hsv_upper_s = threshold_hsv_upper_s.value
        ImageProcessor.threshold_hsv_upper_v = threshold_hsv_upper_v.value

        # ros参数修改回调
        image.add_on_set_parameters_callback(image.callback)
        
        # ros节点配置
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