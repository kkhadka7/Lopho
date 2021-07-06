#Dependencies
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exception
import math
import argparse
from pymavlink import mavutil


### functions

def connectMyCopter():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect
	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()
	
	vehicle = connect(connection_string, wait_ready=True)
	return vehicle

def arm_and_takeoff(target_height):
	while not vehicle.is_armable:
		print("Initializing vehicle ...")
		time.sleep(1)
	print("Vehicle is armable now. ")

	vehicle.mode = VehicleMode("GUIDED")
	while vehicle.mode != 'GUIDED':
		print("Waiting for drone to enter GUIDED mode")
		time.sleep(1)
	print("Vehicle in GUIDED mode now")
	print(vehicle.mode)
	
	vehicle.armed = True
	while not vehicle.armed:
		print("Waiting for vehicle to become armed")
		time.sleep(1)
	print("Vehilce Armed")

	vehicle.simple_takeoff(target_height) # height in meters
	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= 0.95 * target_height:
			break
		time.sleep(1)
	print("Target altitude reached")
	return None
# yaw control function
# this function doesn't work until the drone flies to the same place where it is already located
# Use some dummy function to fly it to current place
# Update 2021 Jun: this bug has been fixed already
def condition_yaw(degrees, relative, cw_acw):
	if relative:
		is_relative = 1 # yaw is relative to direction of travel
	else:
		is_relative = 0 # yaw is absolute angle
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
		0, # confirmation
		degrees, #param 1, yaw in degrees
		0,       #param 2, yaw speed deg/s
		cw_acw,       #param 3, direction -1 ccw, 1 cw
		is_relative, #param 4, relative offset 1, absolute angle 0
		0,0,0
	) #param 5-7 not used
	#send command to vehicle
	vehicle.send_mavlink(msg)
	vehicle.flush()

#dummy function to setup the condition_yaw function
def dummy_yaw_initializer():
	lat=vehicle.location.global_relative_frame.lat
	lon=vehicle.location.global_relative_frame.lon
	alt=vehicle.location.global_relative_frame.alt

	aLocation =LocationGlobalRelative(lat,lon,alt)

	msg = vehicle.message_factory.set_position_target_global_int_encode(
		0,		# time_boot_ms(not used)
		0, 0, 	# target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, #frame
		0b0000111111111000, #type_mask (only speeds enabled)
		aLocation.lat*1e7, #lat int - X position in WGS84Frame in 1e7 * meters
		aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		aLocation.alt, # altitude in meters in AMSL altituce, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
		0, # X velocity in NED frame in m/s
		0, # y velocity in NED frame in m/s
		0, # z velocity in NED frame in m/s
		0, 0, 0, #afx, afy, afz acceleration (not supported yet, ignored in GCS_mavlink)
		0, 0 #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	)
	#send command to vehicle
	vehicle.send_mavlink(msg)
	vehicle.flush()

	
### Main executable
vehicle = connectMyCopter()
arm_and_takeoff(10) # height in meters 

#doesn't require dummy initializer anymore
#dummy_yaw_initializer()
time.sleep(2)

condition_yaw(30,1, 1) # 1 for relative, 1 for CW direction
print("Yawing 30 degrees relative to current position, 343 -> 13")
time.sleep(7)

condition_yaw(0,0,1) # 0 for absolute
print("Yawing to true North")
time.sleep(7)

condition_yaw(270,0,1) # 0 for absolute
print("Yawing to true West")
time.sleep(7)

condition_yaw(30,1,-1) #1 for relative, -1 for ACW direction
print("Yawing 30 degrees anticlock relative to current position, 270 -> 240")
time.sleep(7)


print("Exiting program in 20 seconds ")
time.sleep(20)

vehicle.close()



