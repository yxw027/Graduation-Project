# coding=utf-8
'''
File: TakeoffAndLand.py
Author: Roger Wang
Description: This file is an example for a basic practice to command the drone to 
             takeoff, hover for 10 seconds and land.           
'''

import FlightController
import sys
import time
import math
# import RPi.GPIO as GPIO

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(21,GPIO.OUT)
USE_BIZZ = True
PI = 3.14159
# Create a Vehicle object
# connection_string = '/dev/ttyAMA0'
# connection_string = '127.0.0.1:14550'
connection_string = 'tcp:127.0.0.1:5760'
myCopter = FlightController.Vehicle(FCAddress=connection_string)

# Connect and initialize the vehicle, enable SITL here
if not myCopter.initialize(simulation = True):
    sys.exit(1)

# Try arming the vehicle
timeoutCounter = 0
while not myCopter.arm():
    timeoutCounter += 1
    if timeoutCounter > 3:
        print("Cannot arm the vehicle after 3 retries.")
        sys.exit(1)

# Takeoff
init_taw = myCopter.vehicle.attitude.yaw
print("init taw: ", myCopter.vehicle.attitude.yaw)

if not myCopter.takeoff(0.6):
    sys.exit(1)



# myCopter.send_nav_velocity(5, 0, 0)
# # time.sleep(3)
# print("goto")
# myCopter.goto(0, 5, 0)
# time.sleep(3)
# 测试UAV的飞行轨迹是一个半径5m的圆
print("circle")
cnt = 0
T = 20.0
period = 0.2
R = 1.0
while True:
    v_x = R * 2 * PI / T * math.cos(2 * PI / T * cnt * period)
    v_y = -R * 2 * PI / T * math.sin(2 * PI / T * cnt * period)
    print(v_x, v_y)
    x = R * period * 2 * PI / T * math.cos(2 * PI / T * cnt * period)
    y = -R * period * 2 * PI / T * math.sin(2 * PI / T * cnt * period)
    # myCopter.send_nav_velocity(v_x, v_y, 0)
    myCopter.goto(x, y, 0)
    cnt = cnt + 1
    time.sleep(period)

# Land
timeoutCounter = 0
while not myCopter.land():
    timeoutCounter += 1
    if timeoutCounter > 3:
        print ("Critical: Cannot land the vehicle after 3 retries.")
        sys.exit(1)
        
myCopter.exit()
