# coding=utf-8
from __future__ import print_function

import os
import threading
import math
# import RPi.GPIO as GPIO
from dronekit import *

import FlightController
import object_track
# import detectTest
import speedControl

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(21,GPIO.OUT)
# connection_string = '/dev/ttyAMA0'
connection_string = '127.0.0.1:14550'
# connection_string = 'COM6'
baud_rate = 921600
global right, forward
USE_BIZZ = True


class DetectThread(threading.Thread):
    def __init__(self, name):
        super(DetectThread, self).__init__()
        self.name = name

    def run(self):
        object_track.object_tracker()


def UAV_init(port, height):
    # ***************UAV初始化**********************
    myCopter = FlightController.Vehicle(FCAddress=port)
    # home_alt = my_vehicle.vehicle.location.global_relative_frame.alt
    if not myCopter.initialize(simulation=False):
        return None
    # Try arming the vehicle
    timeoutCounter = 0
    while not myCopter.arm():
        timeoutCounter += 1
        if timeoutCounter > 3:
            print("Cannot arm the vehicle after 3 retries.")
            return None
    before_yaw = myCopter.vehicle.attitude.yaw
    # 声明无人机即将起飞
    # if USE_BIZZ:
    #     for i in range(0, 5):
    #         GPIO.output(21, True)
    #         time.sleep(0.1)
    #         GPIO.output(21, False)
    #         time.sleep(0.1)

    if not myCopter.takeoff(height):
        return None
    after_yaw = myCopter.vehicle.attitude.yaw
    myCopter.goto(0.01, 0, 0)
    myCopter.condition_yaw(math.degrees(abs(after_yaw - before_yaw)), clock_wise = True if after_yaw - before_yaw > 0 else False)
    return myCopter

def fly_circle(vehicle):
    T = 10.0
    period = 0.5
    R = 0.5
    PI = 3.1415
    cnt = 0
    while True:
        v_y = R * 2 * PI / T * math.cos(2 * PI / T * cnt * period)
        v_x = -R * 2 * PI / T * math.sin(2 * PI / T * cnt * period)
        # print(v_x, v_y)
        x = R * period * 2 * PI / T * math.cos(2 * PI / T * cnt * period)
        y = -R * period * 2 * PI / T * math.sin(2 * PI / T * cnt * period)
        vehicle.send_nav_velocity(v_x, v_y, 0)
        cnt = cnt + 1
        time.sleep(0.5)

def UAV_start_task(vehicle):
    # 正式代码
    # global pos_right, pos_forward
    # while not vehicle.fsController.triggered:
    #     pos_right, pos_forward = detectTest.get_offset()
    #     print("x=%.2f, y=%.2f" % (pos_right, pos_forward))
    #     vehicle.goto(pos_right, pos_forward, 0)
    #     time.sleep(3)

    # 以下几行代码是测试UAV能否到达目标
    right = forward = 0
    dex = dey = 0
    ddex = ddey = 0
    last_right = last_forward = 0
    while not vehicle.fsController.triggered:
        # 获得当前位置与目标位置（图像中心）的误差
        ex, ey, EMERGENCE = object_track.get_offset()
        if EMERGENCE:
            break
        if ex != 0 or ey != 0:
            # PD控制
            addright, addforward, dex, dey, ddex, ddey = speedControl.speedControl(ex, ey, dex, dey, ddex, ddey)
            right += addright
            forward += addforward
            if math.fabs((last_forward - forward) / (forward + 0.01)) > 0.2 or \
             math.fabs((last_right - right) / (right + 0.01)) > 0.2:
                print("x=%.2f, y=%.2f" % (right, forward))
                vehicle.send_nav_velocity(forward, right, 0)
            last_right = vehicle.velocity[0]
            last_forward = vehicle.velocity[1]
        else:
            print("Hovering.")
        time.sleep(0.5)


        # last_right = last_forward = 0
        # while not vehicle.fsController.triggered:
        #     # 获得当前位置与目标位置（图像中心）的误差
        #     right, forward, EMERGENCE = object_track.get_offset()
        #     if EMERGENCE:
        #         print("Landing...")
        #         vehicle.land()
        #         break
        #     if math.fabs((last_forward - forward) / (forward + 0.01)) > 0.2 or \
        #             math.fabs((last_right - right) / (right + 0.01)) > 0.2:
        #         print("x=%.2f, y=%.2f" % (right, forward))
        #         # vehicle.goto(pos_right, pos_forward, 0)
        #         vehicle.send_nav_velocity(forward, right, 0)
        #
        #     last_right = vehicle.velocity[0]
        #     last_forward = vehicle.velocity[1]
        #     time.sleep(0.5)



if __name__ == "__main__":
    detectTarget_thread = DetectThread("Target Detect")
    # 设置图像检测线程为守护线程，主程序结束时就退出
    detectTarget_thread.setDaemon(True)
    vehicle = UAV_init(connection_string, 3)
    if not vehicle:
        print("Progress exiting.")
        os._exit(1)
    # vehicle = UAV.connect_UAV(connection_string)
    detectTarget_thread.start()
    time.sleep(5)
    UAV_start_task(vehicle)
    vehicle.land()
    vehicle.exit()

