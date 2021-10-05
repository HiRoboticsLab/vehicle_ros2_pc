# 修改自
# https://github.com/ros2/teleop_twist_keyboard/blob/dashing/teleop_twist_keyboard.py


import sys
from std_msgs.msg import String, Int32MultiArray
import rclpy
import termios
import tty

import os
import json
from vehicle_kinematics.kinematics import Kinematics
from ament_index_python.packages import get_package_share_directory


# 加载运动学节点
# print(dir(Kinematics))
config_file_path = os.path.join(get_package_share_directory('vehicle_kinematics'), 'config', 'vehicle.json')
print(config_file_path)
file = open(config_file_path, 'rb')
config_json = json.load(file)
print(config_json)
kinematics = Kinematics(config_json['wheel_distance'], config_json['wheel_diameter'], config_json['wheel_laps_code'])


msg = """
Must use lowercase
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c
---------------------------
Speed control:
   y    u    i    o
---------------------------
Lighting control:
   h    j    k    l
---------------------------
CTRL-C to quit
"""

moveBindings = {
    'q': (1, 0, 0, 1),
    'w': (1, 0, 0, 0),
    'e': (1, 0, 0, -1),
    'a': (0, 0, 0, 1),
    's': (0, 0, 0, 0),
    'd': (0, 0, 0, -1),
    'z': (-1, 0, 0, -1),
    'x': (-1, 0, 0, 0),
    'c': (-1, 0, 0, 1),
}

adjustBindings = {
    'y': (+0.02, 0.0),
    'u': (-0.02, 0.0),
    'i': (0.0, +0.1),
    'o': (0.0, -0.1),
}

lightBindings = {
    'h': "head",
    'j': "left",
    'k': "both",
    'l': "right",
}

barrierBindings = {
    '[': "open",
    ']': "close"
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('vehicle_ctrl_keyboard')
    # 车轮、车灯控制节点
    pub_wheel = node.create_publisher(Int32MultiArray, 'vehicle/cmd_wheel', 10)
    pub_light = node.create_publisher(String, 'vehicle/cmd_light', 10)
    pub_esp = node.create_publisher(String, 'vehicle/cmd_esp', 10)


    speed = 0.2
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)

            # 移动控制
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

                linear = [x * speed, 0, 0]
                angular = [0, 0, th * turn]

                result = kinematics.forward(linear, angular)

                wheel_msg = Int32MultiArray()
                wheel_msg.data = [int(result[0]), int(result[1])]
                print(int(result[0]), int(result[1]))
                pub_wheel.publish(wheel_msg)

            # 速度控制
            elif key in adjustBindings.keys():
                speed = speed + adjustBindings[key][0]
                turn = turn + adjustBindings[key][1]
                print(vels(speed, turn))
                if (status == 10):
                    print(msg)
                status = (status + 1) % 11

            # 灯光控制
            elif key in lightBindings.keys():
                light_msg = String()
                light_msg.data = '%s' % lightBindings[key]
                pub_light.publish(light_msg)

             # 道闸控制
            elif key in barrierBindings.keys():
                cmd = barrierBindings[key]
                print(cmd)
                data = {}
                data['cmd'] = cmd
                data['id'] = 'null'
                barrier_msg = String()
                barrier_msg.data = str(data)
                pub_esp.publish(barrier_msg)

            # ctrl c 退出
            if (key == '\x03'):
                break

    except Exception as e:
        print(e)

    finally:
        wheel_msg = Int32MultiArray()
        wheel_msg.data = [int(0), int(0)]
        pub_wheel.publish(wheel_msg)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()