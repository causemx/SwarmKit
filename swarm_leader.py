# This is the main function for leader drone.
# Version 2.1

import threading
import time
import netifaces as ni
import logging
import os
import builtins
from core.control import connect, ConnectionType
from swarm.swarm_core import (
    start_SERVER_service, 
    CHECK_network_connection, 
    arm_no_RC,
    air_break,
    return_to_launch,
    wait_for_follower_ready,
    takeoff_and_hover,
    CLIENT_send_immediate_command,
    new_gps_coord_after_offset_inBodyFrame,
    goto_gps_location_relative,
    distance_between_two_gps_coord)

import sys
sys.path.append(os.getcwd())

FORMAT = '%(asctime)s %(filename)s %(levelname)s : %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

# Enter wlan inteface here.
WLAN_INTERFACE = 'wlx04bad60b1ad5'
ROUTER_HOST = '192.168.50.1'

# Get local host IP.
try:
    local_host = ni.ifaddresses(WLAN_INTERFACE)[2][0]['addr']
    host_specifier = local_host[-1]
except ValueError as ve:
    logging.error(str(ve))
    sys.exit("localhost validate error")

# Set log.
"""flight_log_bufsize = 1 # 0 means unbuffered, 1 means line buffered.
flight_log_filename = 'FlightLog' + host_specifier + '_' + '{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '.txt'
flight_log_path = '/home/swarm' + host_specifier + '/Log/'
flight_log_path_filename = flight_log_path + flight_log_filename
flight_log = open(flight_log_path_filename, 'w', flight_log_bufsize)
sys.stdout = flight_log"""

# Specify whether a leader or a follower.
is_leader = True
if is_leader:
    logging.info("This is a leader drone")
    # print('{} - This is a leader drone.'.format(time.ctime()))
    leader_host = local_host
else:
    logging.info("This is a follower drone")

logging.info(f"local_host = {local_host}.")
logging.info(f"This drone specifier = {host_specifier}.")

# Get local host IP.
local_host = ni.ifaddresses(WLAN_INTERFACE)[2][0]['addr']

logging.info(f"local_host = {local_host}.")
host_specifier = local_host[-1]

logging.info(f"This drone is {host_specifier}")

# Reserved port.
# The port number should be exactly the same as that in follower drone.
builtins.port_gps = 60001
builtins.port_status = 60002
builtins.port_immediate_command = 60003
builtins.port_heading = 60004

# Connect to the Vehicle
logging.info("Connecting to vehicle...")
# drone = control.connect('/dev/ttyUSB0', baud=57600, wait_ready=True)
drone = connect(ConnectionType.udp, "127.0.0.1")

while 'drone' not in locals():
    logging.info("Waiting for vehicle connection...")
    time.sleep(1)
builtins.drone = drone
logging.info("Vehicle is connected!")
# Enable safety switch(take effect after reboot pixhawk).
#try:
    # builtins.drone.parameters['BRD_SAFETYENABLE'] = 1 # Enable
builtins.drone.parameters.set('BRD_SAFETYENABLE', 1)
#except AttributeError as ae:
#    logging.error("error: " + str(ae))
#    pass
#vehicle.parameters['BRD_SAFETYENABLE'] = 0 # Disable

# Start server services.
start_SERVER_service(drone, is_leader, local_host)

# Start connection checker. Drone will return home once lost connection.
router_host = '192.168.50.1'
threading.Thread(target=CHECK_network_connection,args=(drone, router_host,),kwargs={'wait_time':10}).start()

# Arm drone without RC.
arm_no_RC(drone)
"""
# IP list:
iris1_host = '192.168.2.101'
iris2_host = '192.168.2.102'
iris3_host = '192.168.2.103'

follower1 = iris2_host
follower2 = iris3_host
follower_host_tuple = (follower1, follower2,)

# Wait untill all followers are ready(armed).
wait_for_follower_ready(follower_host_tuple) # This is a blocking call.

# Get GPS coordinate of leader's launch location.
leader_gps_home = builtins.vehicle.location.global_relative_frame
leader_lat_home = leader_gps_home.lat
leader_lon_home = leader_gps_home.lon
leader_alt_home = leader_gps_home.alt
logging.info(f"Home GPS coordinate: \ {leader_lat_home}, {leader_lon_home}, \ {leader_alt_home}(relative)")

# DOUBLE CHECK the following 4 parameters before each flight mission.
leader_hover_height = 20 # In meter.
leader_fly_distance = 20 # In meters.
leader_aim_heading_direction = builtins.vehicle.heading #(use current) # In degree, 0~360. 90=East

# Fixed parameters.
# fly_follow() parameters for follower1.
follower1_followee = '\''+leader_host+'\'' # The string must contain ''.
follower1_frame_to_followee = '\''+'body'+'\'' # 'body' or 'local'.
# fly_follow() parameters for follower2.
follower2_followee = follower1_followee
follower2_frame_to_followee = follower1_frame_to_followee


# * Formation 1 (Line)
# When taking off, drones are already in this formation.
# Follower 1.
follower1_hover_height = 20 # In meter.
follower1_distance_to_followee = 10 # In meter.
follower1_azimuth_to_followee = 270 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 2.
follower2_hover_height = 20 # In meter.
follower2_distance_to_followee = 14.4 # In meter.
follower2_azimuth_to_followee = 225 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.


# When all members are ready.
# Leader takeoff and hover (in square shape).
threading.Thread(target=takeoff_and_hover, args=(leader_hover_height,)).start()
# Send takeoff command to all followers.
# Immediate command must be in string type.
logging.info(f"Sending immediate command to : {follower1}.")
CLIENT_send_immediate_command(follower1, 'takeoff_and_hover({})'.format(follower1_hover_height))
logging.info(f"Sending immediate command to : {follower2}.")
CLIENT_send_immediate_command(follower2, 'takeoff_and_hover({})'.format(follower2_hover_height))


# Wait for follower ready. Blocking function.
wait_for_follower_ready(follower_host_tuple)

# Get leader current location.
leader_current_gps = builtins.vehicle.location.global_relative_frame
leader_current_lat = leader_current_gps.lat
leader_current_lon = leader_current_gps.lon
leader_current_alt = leader_current_gps.alt
logging.info(f"After taking off and hover, Leader\'s GPS coordinate : \
             lat={leader_current_lat}, lon={leader_current_lon}, alt_relative={leader_current_alt}")
# Get leader current heading.
leader_current_heading = builtins.vehicle.heading
logging.info(f"Leader current heading is {leader_current_heading} degree.")

# Generate a point, leader will fly to this point.
pointA = new_gps_coord_after_offset_inBodyFrame((leader_current_lat,leader_current_lon), leader_fly_distance, leader_current_heading, 0) # 0=Forward, 90=Right, 180=Backward, 270=Left.
logging.info(f"Leader is going to pointA : {pointA}")

# Leader go to new location. Followers fly follow in square shape.
threading.Thread(target=goto_gps_location_relative, args=(pointA[0], pointA[1], leader_hover_height,),kwargs={'groundspeed':1}).start()
# When leader is not at destination location, keep sending follow fly command to followers.
# You can use threading to reduce the delay.
# Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
while ((distance_between_two_gps_coord(
    (builtins.vehicle.location.global_relative_frame.lat, builtins.vehicle.location.global_relative_frame.lon), 
    (pointA[0], pointA[1])) >0.5) or (abs(builtins.vehicle.location.global_relative_frame.alt - leader_hover_height)>0.3)):
    logging.info("Sending command fly_follow() to follower1.")
    CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
    logging.info("Sending command fly_follow() to follower2.")
    CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
    time.sleep(0.5)

# When leader has reached destination, execute air_break().
# At the same time, send air_break command to all followers immediately.
threading.Thread(target=air_break, args=()).start()
for iter_follower in follower_host_tuple:
    print(iter_follower)
    CLIENT_send_immediate_command(iter_follower, 'air_break()')


# * Formation 3 (triangle)
time.sleep(3)
# Shape 3 (triangle).
# Follower 1.
follower1_hover_height = 22 # In meter.
follower1_distance_to_followee = 14.4 # In meter.
follower1_azimuth_to_followee = 225 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 2.
follower2_hover_height = 24 # In meter.
follower2_distance_to_followee = 10 # In meter.
follower2_azimuth_to_followee = 180 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.

# 1) move follower1.
logging.info("Sending command fly_follow() to follower1.")
CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.
# 2) move follower2.
logging.info("Sending command fly_follow() to follower2.")
CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.


# Get leader current location.
leader_current_gps = builtins.vehicle.location.global_relative_frame
leader_current_lat = leader_current_gps.lat
leader_current_lon = leader_current_gps.lon
leader_current_alt = leader_current_gps.alt
logging.info(f"In formation 3 (triangle), leader\'s GPS coordinate : lat={leader_current_lat}, lon={leader_current_lon}, alt_relative={leader_current_alt}")
# Get leader current heading.
leader_current_heading = builtins.vehicle.heading
logging.info(f"Leader current heading is {leader_current_heading} degree.")

# Generate a point, leader will fly to this point.
pointA = new_gps_coord_after_offset_inBodyFrame((leader_current_lat,leader_current_lon), leader_fly_distance, leader_current_heading, 0) # 0=Forward, 90=Right, 180=Backward, 270=Left.
logging.info(f"Leader is going to pointA : {pointA}")

# Leader go to new location.
threading.Thread(target=goto_gps_location_relative, args=(pointA[0], pointA[1], leader_hover_height,),kwargs={'groundspeed':1}).start()
# When leader is not at destination location, keep sending follow fly command to followers.
# You can use threading to reduce the delay.
# Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
while ((distance_between_two_gps_coord((builtins.vehicle.location.global_relative_frame.lat, builtins.vehicle.location.global_relative_frame.lon), (pointA[0], pointA[1])) >0.5) or (abs(builtins.vehicle.location.global_relative_frame.alt - leader_hover_height)>0.3)):
    logging.info("Sending command fly_follow() to follower1.")
    CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
    logging.info("Sending command fly_follow() to follower2.")
    CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
    time.sleep(0.5)

# When leader has reached destination, execute air_break().
# At the same time, send air_break command to all followers immediately.
threading.Thread(target=air_break, args=()).start()
for iter_follower in follower_host_tuple:
    CLIENT_send_immediate_command(iter_follower, 'air_break()')


# * Mission completed, leader and followers go home
# Wait for follower ready.
wait_for_follower_ready(follower_host_tuple)
logging.info("Mission completed. Return home.")

# Follower2 go home.
logging.info("Command follower2 return home.")
CLIENT_send_immediate_command(follower2, 'return_to_launch()')
time.sleep(2)

# Follower1 go home.
logging.info("Command follower1 return home.")
CLIENT_send_immediate_command(follower1, 'return_to_launch()')
time.sleep(2)

# Leader drone go home.
logging.info("Followers have returned home, Leader is returning...")
return_to_launch()
logging.info("Leader has returned home.")
"""