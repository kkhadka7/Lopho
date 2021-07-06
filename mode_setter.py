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
	
	vehicle = connect(connection_string, wait_ready=True)
	return vehicle

### Main executable

vehicle = connectMyCopter()


while not vehicle.is_armable:
	print("Initializing vehicle ...")
	time.sleep(1)
print("Vehicle is armable now. ")
print(vehicle.mode)
vehicle.mode    = VehicleMode("GUIDED")
print(vehicle.mode)
while vehicle.mode != 'GUIDED':
	print("Waiting for drone to enter GUIDED mode")
	print(vehicle.mode)
	time.sleep(1)
print("Vehicle in GUIDED mode now")
print(vehicle.mode)

vehicle.armed = True
while vehicle.armed == False:
	print("Waiting for vehicle to become armed")
	time.sleep(1)
print("Vehilce armed now")

print("Exiting program")
vehicle.close()




