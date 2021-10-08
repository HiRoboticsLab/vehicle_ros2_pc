import rclpy
from rclpy.executors import SingleThreadedExecutor
from .ImageListener import ImageListener


def main(args = None):
    try:
        rclpy.init(args = args)

        image = ImageListener()

        # 形态梯度参数配置
        image.declare_parameter('kernel', 10)
        image.declare_parameter('threshold_low', 240)
        image.declare_parameter('threshold_high', 255)

        (kernel, threshold_low, threshold_high) = image.get_parameters(
            ['kernel', 'threshold_low', 'threshold_high']
        )

        image.kernel = kernel.value
        image.threshold_low = threshold_low.value
        image.threshold_high = threshold_high.value

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

        image.threshold_hsv_lower_h = threshold_hsv_lower_h.value
        image.threshold_hsv_lower_s = threshold_hsv_lower_s.value
        image.threshold_hsv_lower_v = threshold_hsv_lower_v.value
        image.threshold_hsv_upper_h = threshold_hsv_upper_h.value
        image.threshold_hsv_upper_s = threshold_hsv_upper_s.value
        image.threshold_hsv_upper_v = threshold_hsv_upper_v.value

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