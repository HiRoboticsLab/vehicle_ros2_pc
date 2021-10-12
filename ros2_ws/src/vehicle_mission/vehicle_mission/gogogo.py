import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
# 话题
from .topic import Barrier
from .topic import Camera
from .topic import Controller
from .topic import Transform
from .topic import Light
# GUI库
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel
# 线程
import threading
# 时间依赖，用于延时
import time
# easydl
from .utils import detect_traffic_light


gui = None
barrier, camera, controller, transform, light = None, None, None, None, None
image = None
FLAG_AUTO = True


def br_open():
    # 道闸显示屏的id
    barrier.open("2224403829")

def br_close():
    # 道闸显示屏的id
    barrier.close("2224403829")

def go():
    # light.send()
    # controller.send(0.2, 0)
    FLAG_AUTO = True

def stop():
    # light.send()
    controller.send(0, 0)
    FLAG_AUTO = False


list_point = [[0, 0.2, 90], [1.5, 0.2, 0], []]

# 去某个点
def toPoint(x, y, angle):
    cur_x = transform.get_x()
    cur_y = transform.get_y()
    cur_angle = transform.get_angle()

    offset_x = cur_x - x
    offset_y = cur_y - y
    offset_angle = cur_angle - angle

    speed_linear = 0.2
    speed_angular = 0

    # 偏航提供转向力
    if offset_angle > 10:
        # 这里的正负数自己确定一下
        speed_angular = 0.2
    if offset_angle < -10
        # 这里的正负数自己确定一下
        speed_angular = -0.2

    # if abs(offset_x) < 0.05 and abs(offset_y) < 0.05
    # 这里说明达到目的地
        
        

mission_flag = 1
vehicle_camera_image = None
        
# 自动驾驶线程
def auto_drive():
    global mission_flag, vehicle_camera_image
    while True:
        try:
            if FLAG_AUTO:
                # 假设开道闸
                if mission_flag == 0:
                    # 带入id
                    barrier.open()
                    # 睡半秒
                    time.sleep(0.5)
                    # 下一个任务
                    # mission_flag += 1
                # 假设识别红绿灯
                if mission_flag == 1:
                    # 识别码label、和图片长高、以及识别后对图像
                    label, x, y, w, h, image = detect_traffic_light(vehicle_camera_image)
                    # 下一个任务
                    # mission_flag += 1
                # 假设导航
                if mission_flag == 2:
                    # TODO 下面这个函数没有编写完成，目前提供了一个思路
                    # toPoint(x, y, angle)
        

        except Exception as e:
            # traceback.print_exc()
            print(e)



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
        self.label.move(120, 10)
        
        button_barrier_open = QPushButton('道闸开', self)
        button_barrier_open.move(120, 60)
        button_barrier_close = QPushButton('道闸关', self)
        button_barrier_close.move(120, 100)
        button_go = QPushButton('前进', self)
        button_go.move(120, 140)
        button_stop = QPushButton('停止', self)
        button_stop.move(120, 180)

        button_barrier_open.clicked.connect(br_open)
        button_barrier_close.clicked.connect(br_close)
        button_go.clicked.connect(go)
        button_stop.clicked.connect(stop)
        
        self.show()

    def setText(self, str):
        self.label.setText(str)
        self.label.adjustSize()


def callback_camera(image):
    global vehicle_camera_image
    vehicle_camera_image = image
    # 显示图片
    cv2.imshow('camera', vehicle_camera_image)


def main(args = None):
    global gui
    global barrier, camera, controller, transform, light
    try:
        rclpy.init(args = args)

        barrier = Barrier()
        camera = Camera(callback_camera)
        controller = Controller()
        transform = Transform()
        light = Light()
        
        # ros节点配置
        executor = SingleThreadedExecutor()
        executor.add_node(barrier)
        executor.add_node(camera)
        executor.add_node(controller)
        executor.add_node(transform)
        executor.add_node(light)

        # GUI
        app = QApplication(sys.argv)
        gui = App()
        gui.setText("hello world")

        # 自动驾驶线程
        thread = threading.Thread(target = auto_drive, args = ())
        # 为了解决程序退出线程不退出的问题
        thread.setDaemon(True)
        thread.start()

        try:
            executor.spin()
        finally:
            executor.shutdown()
            barrier.destroy_node()
            camera.destroy_node()
            controller.destroy_node()
            transform.destroy_node()
            light.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()