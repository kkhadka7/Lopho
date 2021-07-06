#Dependencies
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exception
import math
import argparse


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
	#baud_rate=57600
	vehicle = connect(connection_string, wait_ready=True)
	return vehicle

### Main executable

vehicle = connectMyCopter()
vehicle.wait_ready('autopilot_version')

## autopilot version
print("Autopilot version: ", vehicle.version)

# find if the firmware supports companion pc to set attitude
print("Support attitude: ", vehicle.capabilities.set_attitude_target_local_ned)

# read actual position
print("Acutal position: ", vehicle.location.global_relative_frame)

# read attitude, roll, pitch, yaw
print("Attitude: ", vehicle.attitude)

# read actual velocity m/s North, East, Down
print("Velocity: ", vehicle.velocity)

# when did we receive last heartbeat
print("Last heartbeat: ", vehicle.last_heartbeat)

# find if vehicle is good to arm
print("Is vehicle armable? ", vehicle.is_armable)

# total vehicle groundspeed
print("Ground speed: ", vehicle.groundspeed) #this is settable

# find current vehilce mode
print("Current Mode: ", vehicle.mode.name)  #this is settable

# check if vehicle is armred
print("Vehicle arm condition: ", vehicle.armed) #this is settable

# check state estimation filter
print("State Estimation Filter Condtion OK: ", vehicle.ekf_ok)


vehicle.close()
