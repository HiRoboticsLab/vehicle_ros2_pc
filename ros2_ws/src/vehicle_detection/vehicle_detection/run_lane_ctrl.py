import rclpy
from rclpy.executors import SingleThreadedExecutor
from .ImageListener import ImageListener
from .Controller import Controller
import math
# GUI库
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel


# 寻线标志位
FLAG_AUTO_LANE = False
# 转弯阈值
threshold_angle = 70
# 控制节点
controller = None
gui = None


def lane_run():
    global FLAG_AUTO_LANE
    FLAG_AUTO_LANE = True

def lane_stop():
    global FLAG_AUTO_LANE, controller
    FLAG_AUTO_LANE = False
    controller.send(0, 0)
    controller.send(0, 0)


class App(QWidget):

    label = None
    
    def __init__(self, callback = None):
        super().__init__()
        self.title = '控制面板'
        self.left = 0
        self.top = 0
        self.width = 320
        self.height = 240

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.label = QLabel(self)
        self.label.width = 160
        self.label.move(80, 10)
        
        button_lane = QPushButton('巡线', self)
        button_lane.move(120, 80)
        button_stop = QPushButton('停止', self)
        button_stop.move(120, 120)
        button_lane.clicked.connect(lane_run)
        button_stop.clicked.connect(lane_stop)
        
        self.show()

    def setText(self, str):
        self.label.setText(str)
        self.label.adjustSize()


def callback_center(center_x, center_y, width, height, image):
    global FLAG_AUTO_LANE, threshold_angle
    side_x = 640 / 2 - center_x
    side_y = 360 - center_y
    angle = math.degrees(math.atan2(side_y, side_x))
    angle = 90 - angle
    result = "center: %s, %s \nwh: %s, %s\nangle: %s" % (center_x, center_y, width, height, angle)
    gui.setText(result)

    if FLAG_AUTO_LANE:
        dir = angle / abs(angle)
        # 微调
        if height >= 115:
            if width > 400:
                controller.send(0.2, 0)
            else:
                if abs(angle) <= 10:
                    controller.send(0.2, 0)
                if abs(angle) > 10 and abs(angle) <= 40:
                    controller.send(0.2, dir * (angle / 90))
                if abs(angle) > 40:
                    controller.send(0.1, dir * 2.0)
        # 大幅度调整
        else:
            if width > 400:
                controller.send(0.2, 0)
            else:
                if abs(angle) <= 10:
                    controller.send(0.2, 0)
                else:
                    controller.send(0.0, dir * 1.5)


def main(args = None):
    global controller, gui
    try:
        rclpy.init(args = args)

        image = ImageListener()
        controller = Controller()

        # 形态梯度参数配置
        image.declare_parameter('kernel', 10)
        image.declare_parameter('threshold_low', 235)
        image.declare_parameter('threshold_high', 255)

        (kernel, threshold_low, threshold_high) = image.get_parameters(
            ['kernel', 'threshold_low', 'threshold_high']
        )

        image.kernel = kernel.value
        image.threshold_low = threshold_low.value
        image.threshold_high = threshold_high.value
        image.callback_center = callback_center

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
        executor.add_node(controller)

        # GUI
        app = QApplication(sys.argv)
        gui = App()

        try:
            executor.spin()
        finally:
            executor.shutdown()
            image.destroy_node()
            controller.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()