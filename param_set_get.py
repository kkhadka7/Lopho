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

gps_type = vehicle.parameters['GPS_TYPE']
print('GPS type is %s'%str(gps_type))
vehicle.parameters['GPS_TYPE']=3
if vehicle.parameters['GPS_TYPE'] != 1:
	vehicle.parameters['GPS_TYPE'] = 1
	gps_type = vehicle.parameters['GPS_TYPE']
print('GPS type is %s'%str(gps_type))
vehicle.close()


