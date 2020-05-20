# coding=utf-8
# ----------------使用仿真跟踪指定轨迹（存储于target.csv中）
# ----------------使用PD控制速度输入---------------------------
from __future__ import print_function

import os
import numpy as np
import csv
from dronekit import *
import pandas as pd
import FlightController
import speedControl

# connection_string = '/dev/ttyAMA0'
connection_string = '127.0.0.1:14550'
# connection_string = 'COM6'
baud_rate = 921600
global right, forward


def get_target_traj():
    with open('target1.csv') as csvfile:
        reader = csv.reader(csvfile)
        return np.array([[float(row[1]), float(row[2])] for row in reader])



def UAV_init(port, height):
    # ***************UAV初始化**********************
    myCopter = FlightController.Vehicle(FCAddress=port)
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

    if not myCopter.takeoff(height):
        return None
    after_yaw = myCopter.vehicle.attitude.yaw
    myCopter.goto(0.01, 0, 0)
    myCopter.condition_yaw(math.degrees(abs(after_yaw - before_yaw)), clock_wise = True if after_yaw - before_yaw > 0 else False)
    return myCopter


def UAV_start_task(vehicle, traj):

    right = forward = 0
    last_right = last_forward = 0
    i = 0
    init = [-2.5,-2]
    dex = dey = 0
    ddex = ddey = 0
    log_file = pd.DataFrame([])
    start_time = time.time()
    while not vehicle.fsController.triggered and i < len(traj):
        current_time = time.time()
        # if (current_time - start_time) / 0.1
        [target_x, target_y] = traj[i] + init
        current_location = vehicle.vehicle.location.local_frame
        pos_x = current_location.east
        pos_y = current_location.north
        off_x = -pos_x + target_x
        off_y = -pos_y + target_y
        # off = np.append(off, [off_x, off_y])

        addright, addforward, dex, dey, ddex, ddey = speedControl.speedControl(off_x, off_y, dex, dey, ddex, ddey)
        right += addright
        forward += addforward
        vehicle.send_nav_velocity(forward, right, 0)
        i = i + 1

        value = [[current_time - start_time,
                  pos_x,
                  pos_y,
                  off_x,
                  off_y,
                  target_x,
                  target_y,
                  vehicle.vehicle.velocity[0],
                  vehicle.vehicle.velocity[1]
                  ]]
        frame = pd.DataFrame(value)
        log_file = log_file.append(frame)
        log_file.to_csv('UAV_offset.csv', index=None)
        time.sleep(0.1)

    # off = off.reshape(-1, 2)
    # name = 'UAV_offset.csv'
    # file = pd.DataFrame(off)
    # file.to_csv(name)

if __name__ == "__main__":

    traj = get_target_traj()
    vehicle = UAV_init(connection_string, 3)
    if not vehicle:
        print("Progress exiting.")
        os._exit(1)
    # vehicle = UAV.connect_UAV(connection_string)
    time.sleep(5)
    UAV_start_task(vehicle, traj)
    vehicle.land()
    vehicle.exit()
