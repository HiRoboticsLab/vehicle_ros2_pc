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


gui = None
barrier, camera, controller, transform, light = None, None, None, None, None


def br_open():
    # 道闸显示屏的id
    barrier.open("2224403829")

def br_close():
    # 道闸显示屏的id
    barrier.close("2224403829")

def go():
    light.send()
    controller.send(0.2, 0)

def stop():
    light.send()
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
    # 显示图片
    cv2.imshow('camera', image)


def main(args = None):
    global gui
    global barrier, camera, controller, laser, light
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