'''
机器人运动分析参考文献
https://mp.weixin.qq.com/s/Mj5iLR_4TKeJiaOVqJf6Vg

    因为
        Vc = WR
        Vleft = W(R - L/2)
        Vright = W(R + L/2)
    所以
        W = (Vright - Vleft) / L
        Vc = (Vleft + Vright) / 2
        Rc = L(Vl + Vr) / 2(Vr - Vl)


每秒脉冲数 = 一圈脉冲数 * 倍频数 /（车轮直径 * PI）* 速度
每米脉冲数 = 一圈脉冲数 * 倍频数 /（车轮直径 * PI）
速度 = 每秒脉冲数 / 每米脉冲数


pip3 install sympy
'''
import sympy as sp
import math

class Kinematics:

    # 参数设定(米)
    wheel_distance = 0.0
    wheel_diameter = 0.0
    wheel_laps_code = 0.0
    last_angle = 0
    # pose_x, pose_y, pose_angle = 0, 0, 0
    pose_x, pose_y = 0, 0
    # 每个脉冲对应的弧度
    tick2rad = 0


    def __init__(self, param_wheel_distance, param_wheel_diameter, param_wheel_laps_code):
        self.wheel_distance = param_wheel_distance
        self.wheel_diameter = param_wheel_diameter
        self.wheel_laps_code = param_wheel_laps_code
        # 经过智能车2倍频而来
        self.tick2rad = (360 / (self.wheel_laps_code * 2)) * math.pi / 180


    # 正向运动学分析
    def forward(self, linear, angular):
        # 智能车内部50ms算一次pwm，所以除以20
        vehicle_speed = linear[0] / 20
        vehicle_angular = angular[2] / 20
        # 声明变量
        vehicle_speed_left, vehicle_speed_right = 0, 0
        vehicle_speed_left = sp.Symbol('vehicle_speed_left')
        vehicle_speed_right = sp.Symbol('vehicle_speed_right')
        # 声明计算公式
        result = sp.solve(
            [
                (vehicle_speed_left + vehicle_speed_right) / 2 - vehicle_speed,                     \
                (vehicle_speed_right - vehicle_speed_left) / self.wheel_distance - vehicle_angular  \
            ],                                                                                      \
            [vehicle_speed_left, vehicle_speed_right]
        )

        # vehicle_code_left = result[vehicle_speed_left] / (math.pi * wheel_diameter) * wheel_laps_code
        # vehicle_code_right = result[vehicle_speed_right] / (math.pi * wheel_diameter) * wheel_laps_code

        vehicle_code_left = result[vehicle_speed_left] / (self.wheel_diameter / 2) / self.tick2rad
        vehicle_code_right = result[vehicle_speed_right] / (self.wheel_diameter / 2) / self.tick2rad

        return float(vehicle_code_left), float(vehicle_code_right)


    # 反向运动学分析
    def backward(self, left, right, angle):
        # 计算行驶距离
        delta_s = ((self.tick2rad * left) + (self.tick2rad * right)) / 2.0 * (self.wheel_diameter / 2)

        # 计算角度变化量
        delta_angle = angle - self.last_angle

        # 现在的姿势
        self.pose_x += delta_s * math.cos((angle + delta_angle) / 180 * math.pi)
        self.pose_y += delta_s * math.sin((angle + delta_angle) / 180 * math.pi)
        # self.pose_angle += delta_angle

        # 更新最后的角度
        self.last_angle = angle
        # return self.pose_x, self.pose_y, self.pose_angle, delta_s, delta_angle
        return float(self.pose_x), float(self.pose_y), float(delta_s), float(delta_angle / 180 * math.pi)
