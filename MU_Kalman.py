#!/usr/bin/python

import smbus
import math
import time
import numpy as np
import kalman
import argparse

# 电源管理寄存器
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
# IMU 灵敏度参数
gyro_sen = 131.0
acc_sen = 16384.0
# IMU 偏移量（更新于 170905）
gx_offset = -502.42
gy_offset = 248.72
gz_offset = 15.75
ax_offset = 140.79
ay_offset = 122.76
az_offset = 1219.97

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr + 1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def get_pitch(x, y, z):
    # 计算 y 轴旋转角度
    radians = math.atan2(x, math.sqrt(y * y + z * z))
    return math.degrees(radians)

def get_roll(x, y, z):
    # 计算 x 轴旋转角度
    radians = -math.atan2(y, z)
    return math.degrees(radians)

bus = smbus.SMBus(1)
address = 0x68
bus.write_byte_data(address, power_mgmt_1, 0)

# 初始化卡尔曼滤波对象用于 RPY（滚转，俯仰，偏航）
Roll = kalman.Kalman()
Pitch = kalman.Kalman()
Yaw = kalman.Kalman()

# 初始时间
time_pre = time.time()

# 获取初始加速度数据
ax_raw = read_word_2c(0x3b)
ay_raw = read_word_2c(0x3d)
az_raw = read_word_2c(0x3f)

ax = (ax_raw - ax_offset) / acc_sen
ay = (ay_raw - ay_offset) / acc_sen
az = (az_raw - az_offset) / acc_sen

# 设置初始卡尔曼角度
Roll.setKalmanAngle(get_roll(ax, ay, az))
Pitch.setKalmanAngle(get_pitch(ax, ay, az))
Yaw.setKalmanAngle(0.)

while True:
    # 读取陀螺仪数据
    gx_raw = read_word_2c(0x43)
    gy_raw = read_word_2c(0x45)
    gz_raw = read_word_2c(0x47)

    gx = (gx_raw - gx_offset) / gyro_sen
    gy = (gy_raw - gy_offset) / gyro_sen
    gz = (gz_raw - gz_offset) / gyro_sen

    # 读取加速度数据
    ax_raw = read_word_2c(0x3b)
    ay_raw = read_word_2c(0x3d)
    az_raw = read_word_2c(0x3f)

    ax = (ax_raw - ax_offset) / acc_sen
    ay = (ay_raw - ay_offset) / acc_sen
    az = (az_raw - az_offset) / acc_sen

    dt = time.time() - time_pre  # 计算时间差

    if abs(gz) < 1:  # 判断是否平稳
        # 如果平稳，输出当前角度
        R = Roll.getKalmanAngle(get_roll(ax, ay, az), gx, dt)
        P = Pitch.getKalmanAngle(get_pitch(ax, ay, az), gy, dt)
        print(f"Roll: {R:.2f}, Pitch: {P:.2f}")
    else:
        # 如果不平稳，每500ms读取一次角度
        time.sleep(0.5)
        R = Roll.getKalmanAngle(get_roll(ax, ay, az), gx, dt)
        P = Pitch.getKalmanAngle(get_pitch(ax, ay, az), gy, dt)
        print(f"Roll: {R:.2f}, Pitch: {P:.2f}")
        # 舵机操作时间400ms
        time.sleep(0.4)

    time_pre = time.time()  # 更新上次时间
