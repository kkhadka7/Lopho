#Dependencies
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
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

### Main executable
vehicle = connectMyCopter()

# command syntax
# cmd1=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
# mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,44.501416,-88.063205,15)
wphome = vehicle.location.global_relative_frame 

spruce = (43.47928210720784,-80.52581474032645,50)
icon = (43.47659442225737,-80.53867323657904,50)
watcollegiate = (43.47946954851475,-80.52909306100452,15)
seven11 = (43.47678594676263,-80.52487878877947,10)
mac = (43.481940937749535,-80.52506781035261,10)
plaza = (43.4718560236277,-80.53481266687248,50)
durga = (43.4795783708637,-80.5240492096826,50)
cmd1 = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wphome.lat,wphome.lon,wphome.alt)
cmd2 = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,*spruce)
cmd3 = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,*watcollegiate)
cmd4 = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,*seven11)
cmd5 = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,*mac)
cmd6 = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0)

#download current list of commands from the drone we are connected to
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

#clear the current list of commands
cmds.clear()

#add in our new commands
cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)
cmds.add(cmd5)
cmds.add(cmd6)

#upload our commands to the drone
vehicle.commands.upload()

#now launch the drone into guided mode, arm and takeoff to 30m height
arm_and_takeoff(30) # height in meters

#switch to AUTO mode
vehicle.mode = VehicleMode('AUTO')
while vehicle.mode != 'AUTO':
	print("Entering AUTO mode ...")
	time.sleep(0.2)

print('AUTO mode entered successfully')

while vehicle.location.global_relative_frame.alt>2:
	print('Drone is executing preloaded mission. But code can still be run in the drone.')
	time.sleep(2)


print("Exiting program")

vehicle.close()



