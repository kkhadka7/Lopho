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

# function to send velocity command to drone +x being heading of the drone (LOCAL Frame NED)
def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,0,0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111, #bitmask --> consider only velocities
		0,0,0, #positions
		vx, vy, vz, #velocities
		0,0,0, #accelerations
		0,0
	)
	vehicle.send_mavlink(msg)
	vehicle.flush()


# function to send velocity command to drone +x being true north (GLOBAL Frame NED)
def send_global_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,0,0,
		mavutil.mavlink.MAV_FRAME_LOCAL_NED,
		0b0000111111000111, #bitmask --> consider only velocities
		0,0,0, #positions
		vx, vy, vz, #velocities
		0,0,0, #accelerations
		0,0
	)
	vehicle.send_mavlink(msg)
	vehicle.flush()


### Main executable
vehicle = connectMyCopter()
arm_and_takeoff(10) # height in meters

# GLOBAL FRAME
print('GLOBAL FRAME')
counter = 0
while counter < 5:
	send_global_ned_velocity(5,0,0)
	print("Moving true north")
	time.sleep(1)
	counter += 1

time.sleep(2)
counter = 0
while counter < 5:
	send_global_ned_velocity(0,-5,0)
	print("Moving true west")
	time.sleep(1)
	counter += 1

time.sleep(5)

# LOCAL Frame
print("Local Frame")
counter = 0
while counter < 5:
	send_local_ned_velocity(5,0,0)
	print("Moving north relative to drone heading")
	time.sleep(1)
	counter += 1

time.sleep(2)
counter = 0
while counter < 5:
	send_local_ned_velocity(0,-5,0)
	print("Moving west relative to drone heading")
	time.sleep(1)
	counter += 1

time.sleep(5)

# UP and down
print("UP and down")
counter = 0
while counter < 5:
	send_local_ned_velocity(0,0,-5)
	print("Moving UP using local ned")
	time.sleep(1)
	counter += 1

time.sleep(2)
counter = 0
while counter < 5:
	send_global_ned_velocity(0,0,5)
	print("Moving DOWN using global ned")
	time.sleep(1)
	counter += 1

while True:
	time.sleep(1)

print("Exiting program")

vehicle.close()



