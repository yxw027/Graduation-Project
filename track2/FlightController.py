from __future__ import print_function

import detectTest
import object_track
import dronekit
from pymavlink import mavutil
import time
import threading
import os
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
import pandas as pd

# Initialization timeout in seconds
INITIALIZE_TIMEOUT = 60
# Mode switching check time period in seconds
SWITCH_MODE_CHECK_TIME = 0.5
# Mode switching timeout in seconds
SWITCH_MODE_TIMEOUT = 4
# Arming check time in seconds
ARM_CHECK_TIME = 0.5
# Arming timeout in seconds
ARM_TIMEOUT = 2
# Taking off target altitude threshold scaler
TAKEOFF_ALT_SCALER = 0.9
# Standard check time in seconds
STD_CHECK_TIME = 1
# Failsafe sleep time in seconds
FS_SLEEP_TIME = 1
# Loiter hover throttle level
LOITER_HOVER_THROTTLE = 1500

'''
Class name: VehicleState
Description: VehicleState is an enumerate class which indicates the 
			 current vehicle state.
'''


class VehicleState(object):
    preFlight = "PREFLIGHT"
    takeoff = "TAKEOFF"
    mission = "MISSION"
    manual = "MANUAL"
    landing = "LANDING"
    landed = "LANDED"


'''
Class name: Custom_DroneKit_Vehicle
Description: Add some custom features to original DroneKit Vehicle class
'''


class Custom_DroneKit_Vehicle(dronekit.Vehicle):
    def __init__(self, *args):
        super(Custom_DroneKit_Vehicle, self).__init__(*args)

        self._ekf_predposhorizrel = False

        @self.on_message('EKF_STATUS_REPORT')
        def listener(self, name, m):
            # boolean: EKF's predicted horizontal position (relative) estimate is good
            # self._ekf_predposhorizrel = (m.flags & ardupilotmega.EKF_PRED_POS_HORIZ_REL) > 0
            self._ekf_predposhorizrel = (m.flags) > 0
            self.notify_attribute_listeners('is_armable', self.is_armable, cache=True)

    '''
	Property name: is_armable
	Description: override the is_armable property in dronekit.Vehicle, intended for
				 indoor OF + RF autonomous missions. 
	'''

    @property
    def is_armable(self):
        # return self.mode != 'INITIALISING' and self.rangefinder.distance and self._ekf_predposhorizrel
        return self.mode != 'INITIALISING' and self._ekf_predposhorizrel


'''
Class name: Vehicle
Description: Main vehicle class
'''


class Vehicle(object):
    def __init__(self, FCAddress=None, baudRate=921600):
        # IP address for SITL simulator. Port 14550 is reserved for GCS.
        # If you're running SITL, make sure mavproxy is running and Port
        # 14551 has been configured as an "--out" port
        self.SITL = "127.0.0.1:14551"
        # self.SITL = "tcp:127.0.0.1:5762"
        # Connection from on-board companion computer to FC
        self.FC = FCAddress
        # Baud rate used to for serial connection
        self.BAUD = baudRate
        # Initialize the vehicle state
        self.STATE = VehicleState.preFlight
        # The vehicle object
        self.vehicle = None
        # The failsafe controller
        self.fsController = None
        self.log_file = None
        self.use_rangefinder = True
        self.log = True
        self.event = threading.Event()
        self.mission_completed = False

    '''
	Property name: attitude
	Description: Current vehicle attitude
				 attitude.roll - Vehicle roll in radians
				 attitude.pitch - Vehicle pitch in radians
				 attitude.yaw - Vehicle yaw in radians
	'''

    @property
    def attitude(self):
        return self.vehicle.attitude

    '''
	Property name: rangefinder
	Description: Current rangefinder read
				 rangefinder.distance - Range finder read in meters
	'''

    @property
    def rangefinder(self):
        return self.vehicle.rangefinder

    '''
    Property name: velocity
    Description: Current velocity[right, forward, down] in meters/sec               
    '''

    @property
    def velocity(self):
        return self.vehicle.velocity

    '''
	Function name: initialize
	Description: Connect to the vehicle with DroneKit and initialize the vehicle object,
				 and start the failsafe controller thread.
	Param: simulation - True to enable simulation, False for real vehicle connection
	Return: True - initialized successfully
			False - initialization not successful
	'''

    def initialize(self, simulation=False):
        # Start the failsafe controller thread
        if not self.fsController:
            self.fsController = FailsafeController(self, self.event)
            self.fsController.start()

        if self.STATE != VehicleState.preFlight:
            print("Err: Connection denied with vehicle state %s." % self.STATE)
            return False

        if simulation:
            connectStr = self.SITL
        else:
            connectStr = self.FC
        print("Connecting to the vehicle on %s with baud rate %d." % (connectStr, self.BAUD))
        self.vehicle = dronekit.connect(connectStr, baud=self.BAUD, wait_ready=True,
                                        vehicle_class=Custom_DroneKit_Vehicle)
        # self.vehicle = dronekit.connect(connectStr, baud=self.BAUD, wait_ready=True)

        if not self.vehicle:
            print("Err: Unable to connect to vehicle.")
            return False
        # self.set_home()
        self.set_global_origin()
        print("Waiting for vehicle to initialize...")
        timeoutCounter = 0

        if self.log:
            self.log_file = StateLogThread(self, self.event)
        # Check if the vehicle is able to arm
        while not self.vehicle.is_armable:
            time.sleep(STD_CHECK_TIME)
            timeoutCounter += 1
            print('vehicle mode = ', self.vehicle.mode.name)
            print('ekf = ', self.vehicle._ekf_predposhorizabs)
            print('rangeFinder=', self.vehicle.rangefinder.distance)
            if timeoutCounter >= (INITIALIZE_TIMEOUT / STD_CHECK_TIME):
                print("Vehicle is not armable.")
                return False
        print("Vehicle initialized successfully, ready for flight.")

        return True

    '''
	Function name: switch_mode
	Description: Switch to a target mode and return whether it was successful.
	Param: targetMode - the mode to be switched, "GUIDED" by default.
	Return: True - successfully switched to target mode
			False - failed to do so
	'''

    def switch_mode(self, targetMode="GUIDED"):
        self.vehicle.mode = dronekit.VehicleMode(targetMode)
        timeoutCounter = 0
        while self.vehicle.mode != dronekit.VehicleMode(targetMode):
            time.sleep(SWITCH_MODE_CHECK_TIME)
            timeoutCounter += 1
            if timeoutCounter >= (SWITCH_MODE_TIMEOUT / SWITCH_MODE_CHECK_TIME):
                return False

        return True

    '''
	Function name: arm
	Description: Arm the vehicle and return whether it was successful.
	Param: checkGUIDED - when set True, if the vehicle were not in GUIDED mode,
						 it would call switch_mode() to switch to GUIDED mode.
	Return: True - armed successfully
			False - failed to do so
	'''

    def arm(self, checkGUIDED=True):
        if checkGUIDED and self.vehicle.mode != dronekit.VehicleMode("GUIDED"):
            self.switch_mode()

        self.vehicle.armed = True
        timeoutCounter = 0
        while not self.vehicle.armed:
            time.sleep(ARM_CHECK_TIME)
            timeoutCounter += 1
            if timeoutCounter >= (ARM_TIMEOUT / ARM_CHECK_TIME):
                return False

        self.STATE = VehicleState.landed
        return True

    '''
	Function name: disarm
	Description: If condition permits, disarm the vehicle.
	Param: force - True to skip all checks and disarm the vehicle
	Return: True - disarmed successfully
			False - disarm denied
	'''

    def disarm(self, force=False):
        if not force:
            if self.STATE != VehicleState.landed:
                print("Err: Cannot disarm in %s state." % self.STATE)
                return False

        self.vehicle.armed = False
        return True

    '''
	Function name: takeoff
	Description: Send takeoff command for the vehicle to a target altitude.
	Param: targetHeight - target height in meters
		   wait_ready - wait until the vehicle has reached the target altitude, True by default
	Return: True - takeoff command sent successfully
			False - cannot send takeoff command
	'''

    def takeoff(self, targetHeight, wait_ready=True, relative=True):
        if self.fsController.triggered:
            return False
        if self.STATE != VehicleState.landed:
            print("Err: Takeoff denied with vehicle state %s." % self.STATE)
            return False

        if relative:
            startHeightGlobal = self.vehicle.rangefinder.distance if self.use_rangefinder else self.vehicle.location.global_relative_frame.alt
            targetHeightGlobal = startHeightGlobal + targetHeight
        else:
            startHeightGlobal = 0
            targetHeightGlobal = targetHeight

        self.STATE = VehicleState.takeoff
        if self.vehicle.mode.name != "GUIDED" or not self.vehicle.armed:
            print("Vehicle is not in GUIDED or armed.")
            print("Mode: ", self.vehicle.mode.name)
            print("Armed: ", self.vehicle.armed)
            return False
        self.vehicle.simple_takeoff(targetHeightGlobal)

        print("Vehicle is taking off!")
        if self.log:
            self.log_file.start()
        while wait_ready:
            if self.STATE != VehicleState.takeoff:
                print("Err: Takeoff terminated unexpectedly with state %s." % self.STATE)
                return False
            print("Current alt: %.2f" % self.vehicle.location.global_relative_frame.alt)
            if self.get_height() - startHeightGlobal >= targetHeight * TAKEOFF_ALT_SCALER:
                print("Reached target altitude")
                self.STATE = VehicleState.mission
                break
            time.sleep(STD_CHECK_TIME)

        return True

    '''
	Function name: land
	Description: Switch the vehicle mode to LAND. It will land the vehicle and 
				 automatically disarm the vehicle once landed.
	Param: force - True to ignore all checks
	Return: True - landing completed
			False - landing command was denied by vehicle
	'''

    def land(self, force=False):
        if not force:
            if self.fsController.triggered:
                return False
            if self.STATE == VehicleState.preFlight \
                    or self.STATE == VehicleState.landing \
                    or self.STATE == VehicleState.landed:
                print("Err: Land denied with vehicle state %s." % self.STATE)
                return False

        # Change the STATE first, prevent triggering failsafe incorrectly
        lastSTATE = self.STATE
        self.STATE = VehicleState.landing

        if self.switch_mode("LAND"):
            print("Landing")
        else:
            print("Err: Failed to switch to LAND.")
            self.STATE = lastSTATE
            return False

        while self.vehicle.armed:
            time.sleep(STD_CHECK_TIME)
        print("Landed successfully.")
        self.STATE = VehicleState.landed
        return True

    '''
	Function name: get_height
	Return: Current vehicle's altitude
	'''

    def get_height(self):
        return self.vehicle.rangefinder.distance if self.use_rangefinder else self.vehicle.location.global_relative_frame.alt

    '''
	Function name: goto
	Description: guide the vehicle to a position relative to the vehicle
	Param: pos_x - Position forward/northward in meters
		   pos_y - Position rightward/eastward in meters
		   pos_z - Position downward in meters (should be negative)
		   relative - True for relative position(forward etc), False for absolute 
					  position(northward etc)
	Return: True - message sent successfully
			False - operation denied
	'''

    def goto(self, pos_x, pos_y, pos_z, relative=True):
        if self.fsController.triggered:
            return False
        if self.STATE != VehicleState.mission:
            print("Err: Goto command denied with vehicle state %s." % self.STATE)
            return False

        # Decide which frame type to be used
        if relative:
            frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
        else:
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED

        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            frame,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            pos_x, pos_y, pos_z,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity(not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)  # yaw, yaw_rate (not used)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        return True

    '''
	Function name: send_nav_velocity
	Description: send_nav_velocity command to vehicle to request it fly in 
				 specified direction
				 Note: From Copter 3.3 the vehicle will stop moving if a new message is not received in approximately 3 seconds.
	Param: velocity_x - forward/northward velocity in m/s
		   velocity_y - rightward/eastward velocity in m/s
		   velocity_z - downward velocity in m/s
		   relative - True for relative velocity(forward etc), False for absolute
		   velocity(northward etc)
	Return: True - message sent successfully
			False - operation denied
	'''

    def send_nav_velocity(self, velocity_x, velocity_y, velocity_z, relative=True):
        if self.fsController.triggered:
            return False
        if self.STATE != VehicleState.mission:
            print("Err: Velocity control denied with vehicle state %s." % self.STATE)
            return False

        # Decide which frame type to be used
        if relative:
            frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
        else:
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            frame,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)  # yaw, yaw_rate (not used)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        return True

    '''
	Function name: condition_yaw
	Description: send condition_yaw MAVLink command to control the vehicle's heading
	Param: heading - vehicle's target heading in degrees,
		   relative - True for relative yaw angle, False for absolute
		   clock_wise - Only meaningful when relative = True.
						True for clock wise, False for counter clock wise
	Return: True - message sent successfully
			False - operation denied
	'''

    def condition_yaw(self, heading, relative=True, clock_wise=True):
        if self.fsController.triggered:
            return False
        if self.STATE != VehicleState.mission:
            print("Err: Yaw control denied with vehicle state %s." % self.STATE)
            return False

        # Yaw command in absolute or relative angle
        if relative:
            isRelative = 1
        else:
            isRelative = 0

        # The vehicle will rotate in cw or ccw by some degrees
        if clock_wise:
            direction = 1  # Degree described by 'heading' will be added to current degree
        else:
            direction = -1  # Degree described by 'heading' will be subscribed from current degree
        if not relative:
            direction = 0
        init_taw = self.vehicle.attitude.yaw
        # create the CONDITION_YAW command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed (not used)
            direction,  # param 3, direction
            isRelative,  # param 4, relative or absolute degrees
            0, 0, 0)  # param 5-7, not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        # while abs(self.vehicle.attitude.yaw - init_taw) < 0.95 * math.radians(heading):
        # 	pass
        # print("Successfully change heading.")
        return True

    '''
	Function name: hover
	Description: command the vehicle to hold its current position. This method
				 will switch the vehicle mode to LOITER and override the throttle
				 level to LOITER_HOVER_THROTTLE
	Param: duration - time for hovering in seconds
	Return: True - message sent successfully
			False - operation denied
	'''

    def hover(self, duration):
        if self.fsController.triggered:
            return False
        if self.STATE != VehicleState.mission:
            print("Err: Hovering denied with vehicle state %s." % self.STATE)
            return False

        # Set RC3(throttle) to the hover level
        self.vehicle.channels.overrides['3'] = LOITER_HOVER_THROTTLE

        # create the LOITER_UNLIM command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # command
            0,  # confirmation
            0, 0, 0, 0,  # param 1-4, not used
            0, 0, 0)  # param 5-7, set 0 for current position
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        print("Hovering")
        time.sleep(duration)

        if not self.fsController.triggered:
            self.switch_mode()

        # Clear channel overrides
        self.vehicle.channels.overrides['3'] = None

        return True

    '''
	Function name: set_home
	Description: Set 'home' to a specific position. This is necessary for guiding the 
				 vehicle in GPS-denied conditions.
				 NOTE that lng and lat can't be 0, otherwise the vehicle will reject
				 this command
	Return: True - command sent successfully
			False - operation denied
	'''

    def set_home(self, lng=0.1, lat=0.1, alt=0.2):
        if self.STATE != VehicleState.preFlight:
            return False
        # # create the SET_HOME command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
            0,  # confirmation
            0,  # param 1, (1=use current location, 0=use specified location)
            0,  # param 2, unused
            0,  # param 3,unused
            0,  # param 4, unused
            lat, lng, alt)  # param 5 ~ 7 latitude, longitude, altitude
        # send command to vehicle

        for x in range(0, 3):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)
        # self.vehicle.flush()
        # cmds = self.vehicle.commands
        # cmds.download()
        # cmds.wait_ready()
        # self.vehicle.home_location = dronekit.LocationGlobal(lng, lat, alt)
        # self.vehicle.location = dronekit.LocationGlobal(10,10,0)
        # print (self.vehicle.home_location)
        print(self.vehicle.location.global_frame)

    def set_global_origin(self):
        """
		Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
		to use local position information without a GPS.
		"""
        target_system = 0
        # target_system = 0   # 0 --> broadcast to everyone
        lattitude = 30
        longitude = 130
        altitude = 1
        f = fifo()
        mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)
        msg = MAV_APM.MAVLink_set_gps_global_origin_message(
            target_system,
            lattitude,
            longitude,
            altitude)
        msg.pack(mav)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()


    '''
	Function name: exit
	Description: Mission completed. Stop all operations
	Return: True - exited successfully
			False - Something wrong happened
	'''

    def exit(self):
        if self.STATE != VehicleState.landed and self.STATE != VehicleState.preFlight:
            print("Err: cannot exit when still in the air! State %s." % self.STATE)
            return False

        self.fsController.join()
        # if self.log:
        #     self.log_file.join()
        self.mission_completed = True
        self.vehicle.close()

        return True


'''
Class name: FailsafeController
Description: A failsafe thread handling failsafe exceptions.
'''


class FailsafeController(threading.Thread):
    def __init__(self, ctrlInstance, event):
        self.instance = ctrlInstance
        self.stoprequest = event
        self.triggered = False
        super(FailsafeController, self).__init__()

    def run(self):
        disarmCounter = 0
        while not self.stoprequest.isSet():
            if self.instance.STATE == VehicleState.mission or self.instance.STATE == VehicleState.takeoff:
                # The vehicle is disarmed unexpectedly
                if not self.instance.vehicle.armed:
                    # Counter was added against latency in 'vehicle.armed'
                    disarmCounter += 1
                    if disarmCounter >= 3:
                        print('Vehicle disarmed unexpectedly.')
                        self.instance.STATE = VehicleState.landed
                else:
                    disarmCounter = 0
                # A failsafe error will trigger the aircraft to switch into LAND or RTL mode
                if self.instance.vehicle.mode == 'LAND' or self.instance.vehicle.mode == 'RTL':
                    self.triggered = True
                    self.instance.vehicle.channels.overrides = {}
                    if self.instance.vehicle.armed:
                        print('Failsafe triggered, mode = LAND or RTL, armed, now landing.')
                        self.instance.STATE = VehicleState.landing
                    else:
                        print('Failsafe triggered, mode = LAND or RTL, disarmed, landed unexpectedly. Now exiting...')
                        self.instance.STATE = VehicleState.landed
                        os._exit(1)

                # If not in GUIDED or AUTO mode, the vehicle is controlled manually
                elif self.instance.STATE != VehicleState.manual and self.instance.vehicle.mode != 'GUIDED' \
                        and self.instance.vehicle.mode != 'AUTO' and self.instance.vehicle.mode != 'LOITER':
                    self.instance.STATE = VehicleState.manual

            if self.triggered and not self.instance.vehicle.armed and self.instance.STATE == VehicleState.landing:
                print('Landed. Now exiting...')
                self.stoprequest.set()
                time.sleep(2)
                os._exit(1)

            time.sleep(FS_SLEEP_TIME)

    def join(self, timeout=None):
        self.stoprequest.set()
        super(FailsafeController, self).join(timeout)


class fifo(object):
    """ A simple buffer """

    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)


class StateLogThread(threading.Thread):
    def __init__(self, ctrlInstance, event):
        self.instance = ctrlInstance
        self.stoprequest = event
        self.triggered = False
        self.column_name = ['time', 'mode', 'height', 'yaw', 'v_forward', 'v_right', 'pos_forward', 'pos_right']
        self.data = []
        super(StateLogThread, self).__init__()

    def run(self):
        print("Start logging.\n")
        ct = time.strftime('%Y-%m-%d-%H%M%S', time.localtime(time.time()))
        path = ct + str('.csv')
        log_file = pd.DataFrame([], columns=self.column_name)
        start_time = time.time()

        while not self.stoprequest.isSet() or not self.instance.mission_completed:
            height = self.instance.get_height()
            current_time  = time.time()
            value = [[current_time - start_time,
                     self.instance.STATE,
                     height,
                     self.instance.vehicle.attitude.yaw,
                     self.instance.vehicle.velocity[0],
                     self.instance.vehicle.velocity[1],
                     # object_track.get_offset()[0],
                     # object_track.get_offset()[1]
                     self.instance.vehicle.location.local_frame.east,
                     self.instance.vehicle.location.local_frame.north]]
            frame = pd.DataFrame(value, columns=self.column_name)
            log_file = log_file.append(frame)
            log_file.to_csv(path, index=None)
            time.sleep(0.5)
        print("Write Logs:%s" % ct)

    def join(self, timeout=None):
        self.stoprequest.set()
        super(StateLogThread, self).join(timeout)

