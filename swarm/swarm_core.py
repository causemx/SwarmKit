# This file defines all functions that is needed in both formation_main_leader.py and formation_main_follower.py

import time
import math
import geopy
import socket
import threading
import os
from math import asin, atan2, cos, degrees, radians, sin
from pymavlink import mavutil
from geopy.distance import geodesic
from core.control import VehicleMode, LocationGlobalRelative

# MAVLink Parameters to specify coordinate frame.
# 1) MAV_FRAME_LOCAL_NED:
# Positions are relative to the vehicle's home position in the North, East, Down (NED) frame. Use this to specify a position x metres north, y metres east and (-) z metres above the home position.
# Velocity directions are in the North, East, Down (NED) frame.

# 2) MAV_FRAME_LOCAL_OFFSET_NED:
# Positions are relative to the current vehicle position in the North, East, Down (NED) frame. Use this to specify a position x metres north, y metres east and (-) z metres of the current vehicle position.
# Velocity directions are in the North, East, Down (NED) frame.

# 3) MAV_FRAME_BODY_OFFSET_NED:
# Positions are relative to the current vehicle position in a frame based on the vehicle's current heading. Use this to specify a position x metres forward from the current vehicle position, y metres to the right, and z metres down (forward, right and down are "positive" values).
# Velocity directions are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).

# 4) MAV_FRAME_BODY_NED:
# Positions are relative to the vehicle's home position in the North, East, Down (NED) frame. Use this to specify a position x metres north, y metres east and (-) z metres above the home position.
# Velocity directions are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).


#=============================================================

# Check connections to router.
def CHECK_network_connection(drone, router_host, wait_time=None):
    print('{} - CHECK_network_connection({}) is started.'.format(time.ctime(), router_host))
    if wait_time is None:
        wait_time = 10 # Default wait time is 10 seconds.
    down_counter = 0
    while True:
        response = os.system('ping -c 1 ' + router_host)
        if response==0: # Link is OK.
            print('{} - Connection to router is OK. Check again in {} seconds'.format(time.ctime(), wait_time))
            down_counter = 0 # Once connection is OK, reset counter.
            time.sleep(wait_time) # Check again in wait_time seconds.
            continue # Return back to the beginning of the while loop.
        else: # Link is down.
            down_counter += 1
            print('{} - Connection to router is DOWN for {} times.'.format(time.ctime(), down_counter))
            if down_counter > 5:
                print('{} - Reached maximum down times.'.format(time.ctime()))
                print('{} - Vehicle is returning home...'.format(time.ctime()))
                drone.mode = VehicleMode('BRAKE')
                break # Terminate while loop.
            else: # Have not reached max down times.
                print('{} - Check again in 1 seconds'.format(time.ctime()))
                time.sleep(1) # Check again in 2 seconds.

#=============================================================

# This function start server services.
# Be sure define the ports as global variables before call this function.
def start_SERVER_service(drone, is_leader, local_host):
    # 1) Start send gps coordinate service.
    threading.Thread(target=SERVER_send_gps_coordinate, args=(drone, local_host,)).start()
    print('{} - Thread SERVER_send_gps_coordinate is started!'.format(time.ctime()))
    # 2) Start send heading direction service.
    threading.Thread(target=SERVER_send_heading_direction, args=(drone, local_host,)).start()
    print('{} - Thread SERVER_send_heading_direction is started!'.format(time.ctime()))
    # 3) Start send follower status command.
    #    Be sure you have decleared a global variable status_waitForCommand.
    threading.Thread(target=SERVER_send_status, args=(local_host,)).start()
    print('{} - Thread SERVER_send_status has started!'.format(time.ctime()))
    # Leader drone does not need the following service.
    if not is_leader:
        # 4) Start SERVER_receive_and_execute_immediate_command service.
        # Be sure you have decleared a global variable status_waitForCommand.
        threading.Thread(target=SERVER_receive_and_execute_immediate_command, args=(local_host,)).start()
        print('{} - Thread SERVER_receive_and_execute_leader_immediate_command has been started!'.format(time.ctime()))

#=============================================================

# This is a server to send gps message.
# TO START IT:
# threading.Thread(target=SERVER_send_gps_coordinate, args=(local_host,)).start().start()
# print(' Thread thread_send_gps has started!')
def SERVER_send_gps_coordinate(drone, local_host):
    global port_gps
    # Create a socket object
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to the port
    msg_socket.bind((local_host, port_gps))
    msg_socket.listen(5)                 # Now wait for client connection.
    print('{} - SERVER_send_gps_coordinate() is started!'.format(time.ctime()))
    while True:
        # msg_socket.accept() will block while loop until the connection with client is established.
        client_connection, client_address = msg_socket.accept() # Establish connection with client.
        print('{} - Received GPS coordinate request from {}.'.format(time.ctime(),client_address))
        # Send message to client.
        # Get current GPS coordinate, compare with destination GPS coordinate.
        
        current_gps_coordinate = drone.location.global_relative_frame
        current_lat = current_gps_coordinate.lat
        current_lon = current_gps_coordinate.lon
        current_alt = current_gps_coordinate.alt

        current_lat_str = '{:.7f}'.format(current_lat)
        current_lon_str = '{:.7f}'.format(current_lon)
        current_alt_str = '{:.7f}'.format(current_alt)
        gps_msg_str = current_lat_str + ',' + current_lon_str + ',' + current_alt_str
        client_connection.send(gps_msg_str.encode())
        # Socket is destroyed when message has been sent.
        client_connection.close()

#=============================================================

# This is a server to send vehicle heading direction information.
# TO START IT:
# threading.Thread(target=SERVER_send_heading_direction, args=(local_host,)).start()
# print(' Thread SERVER_send_heading_direction() has started!')
def SERVER_send_heading_direction(drone, local_host):
    global port_heading
    # Create a socket object
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to the port
    msg_socket.bind((local_host, port_heading))
    msg_socket.listen(5)
    print('{} - SERVER_send_heading_direction() is started!'.format(time.ctime()))
    while True:
        # msg_socket.accept() will block while loop until the connection with client is established.
        client_connection, client_address = msg_socket.accept() # Establish connection with client.
        print('{} - Received heading direction request from {}.'.format(time.ctime(), client_address))
        # Send message to client.
        # Get current heading.
        heading = drone.heading
        current_heading_str = str(heading)
        client_connection.send(current_heading_str.encode())
        # Socket is destroyed when message has been sent.
        client_connection.close()

#=============================================================

# This is a server to receive leader drone's command message.
# CAUTION: This function will execute commands from any host. Check mechanism can be added.
# TO START IT:
# threading.Thread(target=SERVER_receive_and_execute_immediate_command, args=(local_host,)).start()

def SERVER_receive_and_execute_immediate_command(local_host):
    global port_immediate_command
    global status_waitForCommand
    # Create a socket object
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to the port
    msg_socket.bind((local_host, port_immediate_command))
    msg_socket.listen(5)                 # Now wait for client connection.
    print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))
    while True:
        # msg_socket.accept() will block while loop until the connection with client is established.
        client_connection, client_address = msg_socket.accept() # Establish connection with client.
        print('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))
        # Receive message.
        immediate_command_str = client_connection.recv(1024)
        immediate_command_str = immediate_command_str.decode("utf-8")
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        # If command is 'break', execute immediately, regardless of the status of follower drone.
        if immediate_command_str == 'air_break()':
            # Execute received command.
            exec(immediate_command_str)
            # When command is executed, change status to 'wait for command'.
            status_waitForCommand = True
            print('{} - Immediate command \'{}\' is finished!'.format(time.ctime(), immediate_command_str))
        # If command is not 'Break', and status_waitForCommand is true, execute command immediately.
        else: # immediate_command_str is not 'air_break()'
            if status_waitForCommand is True:
                # Change status_waitForCommand to False to block other calls.
                status_waitForCommand = False
                # Execute immediate command.
                exec(immediate_command_str)
                # Change status_waitForCommand to True to enable other calls.
                status_waitForCommand = True
                print('{} - Immediate command \'{}\' is finished!'.format(time.ctime(), immediate_command_str))
            else: # status_waitForCommand == False:
                print('{} - Omit immediate command \'{}\', because status_waitForCommand is False!'.format(time.ctime(), immediate_command_str))
        # Socket is destroyed when message has been sent.
        client_connection.close()

#=============================================================

# This is a server to send status_waitForCommand to requester.
# TO START IT:
# Declare a global variable status_waitForCommand in main function.
# threading.Thread(target=SERVER_send_status, args=(local_host,)).start()
# print(' Thread SERVER_send_status has started!')
def SERVER_send_status(local_host):
    global status_waitForCommand
    global port_status
    # Create a socket object
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to the port
    msg_socket.bind((local_host, port_status))
    msg_socket.listen(5)                 # Now wait for client connection.
    print('{} - SERVER_send_status() is started!'.format(time.ctime()))
    while True:
        # msg_socket.accept() will block while loop until the connection with client is established.
        client_connection, client_address = msg_socket.accept() # Establish connection with client.
        print('{} - Received follower status request from {}.'.format(time.ctime(), client_address))
        # Send message to client.
        str_status_waitForCommand = str(int(status_waitForCommand)).encode()
        client_connection.send(str_status_waitForCommand)
        # Socket is destroyed when message has been sent.
        client_connection.close()

#=============================================================

# This is a client to send immediate command to remote host.
def CLIENT_send_immediate_command(remote_host, immediate_command_str):
    global port_immediate_command
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host, port_immediate_command))
        client_socket.send(immediate_command_str.encode())
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_send_immediate_command({}, {}) is not executed!'.format(time.ctime(), remote_host, immediate_command_str))
        return

#=============================================================

# This is a client to request remote host's status.
def CLIENT_request_status(remote_host):
    global port_status
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host, port_status))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_request_status({}) is not executed!'.format(time.ctime(), remote_host))
        return False
    status_msg_str = client_socket.recv(1024)
    return bool(int(status_msg_str))

#=============================================================

# This is a client to receive remote host's gps coordinate.
def CLIENT_request_gps(remote_host):
    global port_gps
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host, port_gps))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_request_gps({}) is not executed!'.format(time.ctime(), remote_host))
        return None, None, None
    gps_msg_str = client_socket.recv(1024)
    # Return lat, lon, and alt
    lat, lon, alt = (gps_msg_str.decode("utf-8")).split(',')
    return float(lat), float(lon), float(alt)

#=============================================================

# This is a client to receive remote host's gps coordinate.
def CLIENT_request_heading_direction(remote_host):
    global port_heading
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host, port_heading))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_request_heading_direction({}) is not executed!'.format(time.ctime(), remote_host))
        return None
    heading_msg_str = client_socket.recv(1024)
    return int(heading_msg_str)

#=============================================================

# This function should run on leader drone.
def wait_for_follower_ready(follower_host_tuple):
    all_follower_status = []
    for follower_host in follower_host_tuple:
        iter_follower_status = CLIENT_request_status(follower_host)
        all_follower_status.append(iter_follower_status)
    while not all(all_follower_status): # If there is anyone who is False.
        for i in range(len(follower_host_tuple)):
            if not all_follower_status[i]:
                print('{} - Host {} is not ready.'.format(time.ctime(), follower_host_tuple[i]))
        print('{} - Wait for 1 second.'.format(time.ctime()))
        time.sleep(1)
        # Reset status.
        all_follower_status = []
        # Check all followers' status again.
        for follower_host in follower_host_tuple:
            iter_follower_status = CLIENT_request_status(follower_host)
            all_follower_status.append(iter_follower_status)

#=============================================================

# The function send_ned_velocity() below generates a SET_POSITION_TARGET_LOCAL_NED MAVLink message. It directly specify the speed components of the vehicle in the MAV_FRAME_LOCAL_NED frame.
# Velocity is relative to the vehicle's home position.
# Velocity directions are in the North, East, Down (NED) frame.
# The message is re-sent every second for the specified duration.
def send_local_ned_velocity(drone, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    print('\n')
    print('{} - Calling function send_local_ned_velocity(Vx={}, Vy={}, Vz={}, Duration={})'.format(time.ctime(), velocity_x, velocity_y, velocity_z, duration))
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, int(math.ceil(duration))):
        drone.send_mavlink(msg)
        print('{} - Local NED Velocity command is sent! Vx={}, Vy={}, Vz={}'.format(time.ctime(), velocity_x, velocity_y, velocity_z))
        print('{} - Duration = {} seconds'.format(time.ctime(), x+1))
        time.sleep(1)
    # get_vehicle_state(drone)
        print('\n')

#===================================================

# The function send_body_frame_velocity() below generates a SET_POSITION_TARGET_LOCAL_NED MAVLink message. It directly specify the speed components of the vehicle in the MAV_FRAME_BODY_OFFSET_NED frame.
# Velocity is relative to the current vehicle heading.
# Use this to specify the speed forward, right and down (or the opposite if you use negative values).
# The message is re-sent every second for the specified duration.
def send_body_frame_velocity(drone, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    print('\n')
    print('{} - Calling function send_body_frame_velocity(Vx={}, Vy={}, Vz={}, Duration={})'.format(time.ctime(), velocity_x, velocity_y, velocity_z, duration))
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, int(math.ceil(duration))):
        drone.send_mavlink(msg)
        print('{} - Body Frame Velocity command is sent! Vx={}, Vy={}, Vz={}'.format(time.ctime(), velocity_x, velocity_y, velocity_z))
        print('{} - Duration = {} seconds'.format(time.ctime(), x+1))
        time.sleep(1)
        # get_vehicle_state(drone)
        print('\n')

#===================================================

# The function send_local_ned_position() below generates a SET_POSITION_TARGET_LOCAL_NED MAVLink message. It directly specify the position components of the vehicle in the MAV_FRAME_LOCAL_NED frame.
# Position is relative to the vehicle's home position (launch location).
# Position directions are in the North, East, Down (NED) frame.
# The function will wait for an estimated time to finish the moving.
def move_inLocalFrame(drone, north, east, down, groundspeed):
    print('\n')
    print('{} - Calling function move_inLocalFrame(North={}, East={}, Down={}, groundspeed={})'.format(time.ctime(), north, east, down, groundspeed))
    # Time estimation.
    estimatedGroundDistance = math.sqrt(north**2 + east**2)
    if groundspeed > 0:
        estimatedHorizontalFlightTime = estimatedGroundDistance / groundspeed
    else:
        estimatedHorizontalFlightTime = 1
    if not down: # if down is zero
        # We only need to consider the time needed for level flight.
        estimatedTime = estimatedHorizontalFlightTime
        print('{} - 2-Dimension Horizontal flight, estimatied flight time is : {}'.format(time.ctime(), estimatedTime))
    else: # if down is not zero
        estimatedTime = estimatedHorizontalFlightTime + 5
        print('{} - 3-Dimension fight, estimatied flight time is : {}'.format(time.ctime(), estimatedTime))

    # Generate MAVLink message.
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only position enabled)
        north, east, down, # north, east, down positions in meters.
        0, 0, 0, # x, y, z velocity (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # Set ground speed.
    drone.groundspeed = groundspeed
    print('{} - Groundspeed is set to {} m/s.'.format(time.ctime(), drone.groundspeed))

    # Send MAVLink message.
    drone.send_mavlink(msg)

    # Wait estimated time for command to be fully executed.
    for t in range(0, int(math.ceil(estimatedTime))):
        time.sleep(1)
        print('{} - Executed move_inLocalFrame(North={}, East={}, Down={}, groundspeed={}) for {} seconds.'.format(time.ctime(), north, east, down, groundspeed, t+1))
        # get_vehicle_state(drone)
        print('\n')

#===================================================

# The function move_inBodyFrame() below generates a SET_POSITION_TARGET_LOCAL_NED MAVLink message. It directly specify the position components of the vehicle in the MAV_FRAME_LOCAL_NED frame.
# Position is relative to the vehicle's current position.
# Position forward, right and down are "positive" values.
# IMPORTANT: Drone will turn its head toward to travel direction.
# The function will wait for an estimated time to finish the moving.
def move_inBodyFrame(drone, forward, right, down, groundspeed):
    print('\n')
    print('{} - Calling function move_inBodyFrame(forward={}, right={}, down={}, groundspeed={})'.format(time.ctime(), forward, right, down, groundspeed))
    # Time estimation.
    estimatedGroundDistance = math.sqrt(forward**2 + right**2)
    if groundspeed > 0:
        estimatedHorizontalFlightTime = estimatedGroundDistance / groundspeed
    else:
        estimatedHorizontalFlightTime = 1
    if not down: # if down is zero
        # We only need to consider the time needed for level flight.
        estimatedTime = estimatedHorizontalFlightTime
        print('{} - 2-Dimension Horizontal flight, estimatied flight time is : {}'.format(time.ctime(), estimatedTime))
    else: # if down is not zero
        estimatedTime = estimatedHorizontalFlightTime + 5
        print('{} - 3-Dimension fight, estimatied flight time is : {}'.format(time.ctime(), estimatedTime))
    
    # Generate MAVLink message.
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only position enabled)
        forward, right, down, # positions in meters.
        0, 0, 0, # x, y, z velocity (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # Set ground speed.
    drone.groundspeed = groundspeed
    print('{} - Groundspeed is set to {} m/s.'.format(time.ctime(), drone.groundspeed))

    # Send MAVLink message.
    drone.send_mavlink(msg)

    # Wait estimated time for command to be fully executed.
    for t in range(0, int(math.ceil(estimatedTime))):
        time.sleep(1)
        print('{} - Executed move_inBodyFrame(forward={}, right={}, down={}, groundspeed={}) for {} seconds.'.format(time.ctime(), forward, right, down, groundspeed, t+1))
        # get_vehicle_state(drone)
        print('\n')

#===================================================

# Go to specified GPS coordinate.
# lat: Latitude.
# lon: Longitude.
# alt: Altitude in meters(relative to the home location).
def goto_gps_location_relative(drone, lat, lon, alt, groundspeed=None):
    print('\n')
    print('{} - Calling goto_gps_location_relative(lat={}, lon={}, alt={}, groundspeed={}).'.format(time.ctime(), lat, lon, alt, groundspeed))
    destination = LocationGlobalRelative(lat, lon, alt)
    print('{} - Before calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
    # get_vehicle_state(drone)
    # Get current GPS coordinate, compare with destination GPS coordinate.
    current_lat = drone.location.global_relative_frame.lat
    current_lon = drone.location.global_relative_frame.lon
    current_alt = drone.location.global_relative_frame.alt
    # Wait until reach destination.
    while ((distance_between_two_gps_coord((current_lat,current_lon), (lat,lon)) >0.5) or (abs(current_alt-alt)>0.3)):
        # Execute fly command.
        drone.simple_goto(destination, groundspeed=groundspeed)
        # wait for one second.
        time.sleep(0.5)
        # Check current GPS coordinate, compare with destination GPS coordinate.
        current_lat = drone.location.global_relative_frame.lat
        current_lon = drone.location.global_relative_frame.lon
        current_alt = drone.location.global_relative_frame.alt
        # print('{} - Horizontal distance to destination: {} m.'.format(time.ctime(), distance_between_two_gps_coord((current_lat,current_lon), (lat,lon))))
        # print('{} - Perpendicular distance to destination: {} m.'.format(time.ctime(), current_alt-alt))
    # When finishe, check vehicle status.
    # print('{} - After calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
    # get_vehicle_state(drone)


# The vehicle "yaw" is the direction that the vehicle is facing in the horizontal plane.
# On Copter this yaw need not be the direction of travel (though it is by default).
# The yaw will return to the default (facing direction of travel) after you set the mode or change the command used for controlling movement.
# At time of writing there is no safe way to return to the default yaw "face direction of travel" behaviour.
# After taking off, yaw commands are ignored until the first "movement" command has been received. If you need to yaw immediately following takeoff then send a command to "move" to your current position
def set_yaw(drone, yaw_inDegree, bool_isRelative):
    print('\n')
    print('{} - Calling function set_yaw(yaw_inDegree={}, bool_isRelative={}).'.format(time.ctime(), yaw_inDegree, bool_isRelative))
    # Do not pass True of False into msg, just in case the conversion is unpredictable.
    if bool_isRelative:
        is_relative = 1
        print('{} - The degree to turn is relative to current heading.'.format(time.ctime()))
        degreeToTurn = yaw_inDegree
        if degreeToTurn > 180:
            degreeToTurn = 360 - degreeToTurn
        estimatedTime = degreeToTurn/30.0 + 1 # Upon testing, the turning speed is 30 degree/second. Add one more second.
        print('{} - Absolute degree to turn is {} degree. Estimated time is {} seconds.'.format(time.ctime(), degreeToTurn, estimatedTime))
    else:
        is_relative = 0
        print('{} - The target degree is absolute degree[0~360](0=North, 90=East).'.format(time.ctime()))
        currentHeading = drone.heading
        print('{} - Current heading is {} degree.'.format(time.ctime(), currentHeading))
        print('{} - Target heading is {} degree.'.format(time.ctime(), yaw_inDegree))
        degreeToTurn = abs(yaw_inDegree - drone.heading)
        if degreeToTurn > 180:
            degreeToTurn = 360 - degreeToTurn
        estimatedTime = degreeToTurn/30.0 + 1 # Upon testing, the turning speed is 30 degree/second. Add one more second.
        print('{} - Absolute degree to turn is {} degree. Estimated time is {} seconds.'.format(time.ctime(), degreeToTurn, estimatedTime))
    
    # create the CONDITION_YAW command using command_long_encode()
    msg = drone.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        yaw_inDegree,  # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, if set to 0, yaw is an absolute direction[0-360](0=north, 90=east); if set to 1, yaw is a relative degree to the current yaw direction.
        0, 0, 0)    # param 5 ~ 7 not used
    
    # Send MAVLink message.
    drone.send_mavlink(msg)
    
    # Wait sort of time for the command to be fully executed.
    for t in range(0, int(math.ceil(estimatedTime))):
        time.sleep(1)
        print('{} - Executed set_yaw(yaw_inDegree={}, bool_isRelative={}) for {} seconds.'.format(time.ctime(), yaw_inDegree, bool_isRelative, t+1))
        # get_vehicle_state(drone)
        print('\n')

#===================================================

# Calculate new gps coordinate given one point(lat, lon), direction(bearing), and distance. A bearing of 90 degrees corresponds to East, 180 degrees is South, and so on.
def new_gps_coord_after_offset_inLocalFrame(original_gps_coord, displacement, rotation_degree):
    vincentyDistance = geopy.distance.VincentyDistance(meters = displacement)
    original_point = geopy.Point(original_gps_coord[0], original_gps_coord[1])
    new_gps_coord = vincentyDistance.destination(point=original_point, bearing=rotation_degree)
    new_gps_lat = new_gps_coord.latitude
    new_gps_lon = new_gps_coord.longitude
    # If convert float to decimal, round will be accurate, but will take 50% more time. Not necessary.
    #new_gps_lat = decimal.Decimal(new_gps_lat)
    #new_gps_lon = decimal.Decimal(new_gps_lon)
    return (round(new_gps_lat, 7), round(new_gps_lon, 7))


#===================================================

# Calculate new gps coordinate given one point(lat, lon), direction(bearing), and distance. A bearing of 90 degrees corresponds to East, 180 degrees is South, and so on.
def new_gps_coord_after_offset_inBodyFrame(original_gps_coord, displacement, current_heading, rotation_degree_relative):
    # current_heading is in degree, North = 0, East = 90.
    # Get rotation degree in local frame.
    rotation_degree_absolute = rotation_degree_relative + current_heading
    if rotation_degree_absolute >= 360:
        rotation_degree_absolute -= 360
    vincentyDistance = geopy.distance.distance(kilometers=displacement)
    original_point = geopy.Point(original_gps_coord[0], original_gps_coord[1])
    new_gps_coord = vincentyDistance.destination(point=original_point, bearing=rotation_degree_absolute)
    new_gps_lat = new_gps_coord.latitude
    new_gps_lon = new_gps_coord.longitude
    # If convert float to decimal, round will be accurate, but will take 50% more time. Not necessary.
    #new_gps_lat = decimal.Decimal(new_gps_lat)
    #new_gps_lon = decimal.Decimal(new_gps_lon)
    return (round(new_gps_lat, 7), round(new_gps_lon, 7))

#===================================================

def get_point_at_distance(original_gps_coord, displacement, current_heading, R=6371):
    """
    original_gps_coord: original gps coordiantion(lat, lon)
    displacement: target distance from initial (meters)
    current_heading: (true) heading in degrees
    R: optional radius of sphere, defaults to mean radius of earth

    Returns new lat/lon coordinate {d}km from initial, in degrees
    """
    lat1 = radians(original_gps_coord[0])
    lon1 = radians(original_gps_coord[1])
    a = radians(current_heading)
    lat2 = asin(sin(lat1) * cos(displacement/R) + cos(lat1) * sin(displacement/R) * cos(a))
    lon2 = lon1 + atan2(
        sin(a) * sin(displacement/R) * cos(lat1),
        cos(displacement/R) - sin(lat1) * sin(lat2)
    )
    return (degrees(lat2), degrees(lon2),)

#===================================================

# Calculate the distance between two gps coordinate. Return distance in meters.
# 2D.
def distance_between_two_gps_coord(point1, point2):
    distance = geodesic(point1, point2).meters
    return distance

#===================================================

def preArm_override(drone):
    # drone.channels['1'] : Roll
    # drone.channels['2'] : Pitch
    # drone.channels['3'] : Throttle
    # drone.channels['4'] : Yaw
    # If arm without Radio Contoller(RC is turned off), the value of each channel will be 0.
    # The failsafe check requires the throttle value(drone.vhannels['3']) to be above failsafe value.
    # value(drone.parameters.get('FS_THR_VALUE')).
    # To bypass throttle failsafe check, you need to override channel 3.
    # The range of FS_THR_VALUE is (925 ~ 1100), so we can set channel 3 to 1100.
    drone.channels.overrides['3'] = 1100
    # Caution: Do not set channel 3 any higher value, or the vehicle will thrust when armed.

#===================================================

def arm_no_RC(drone):
    # Override RC channel 3, which is the throttle channel.
    # preArm_override(drone)
    
    # Wait for 3 seconds after overriding the throttle channel. Make sure the value is sent to pixhawk.
    time.sleep(3)
    
    # Wait till the vehicle is armable.
    while not drone.is_armable:
        print('{} - Vehicle is not armable, waiting for vehicle to initialise...'.format(time.ctime()))
        time.sleep(1)
    
    # When vehicle is armable, change mode to GUIDED and try to arm it.
    print('{} - Arming motors...'.format(time.ctime()))

    # Copter should arm in GUIDED mode
    drone.mode = VehicleMode('GUIDED')
    # Wait for 3 seconds after mode change. Make sure the value is sent.
    time.sleep(3)
    # Check if the vehicle mode is GUIDED.
    print('{} - Vehicle mode is changed to {}'.format(time.ctime(), drone.mode.name))

    # Try to arm drone. The first time will be probably failed. It will initialize APM. After initializing, the second time will be a succeed.
    drone.armed = True
    # If the first time arming failed, arm again till success.
    while not drone.armed:
        print('{} - Vehicle is not armed, try to arm vehicle again...'.format(time.ctime()))
        time.sleep(3)
        drone.armed = True

#===================================================

def air_break(drone):
    if drone.armed:
        print('\n')
        print('{} - Calling function air_break().'.format(time.ctime()))
        
        msg = drone.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            0, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # Send message one time, then check the speed, if not stop, send again.
        print('{} - Sending air break command first time.'.format(time.ctime()))
        drone.send_mavlink(msg)
        # get_vehicle_state(drone)
        while ((drone.velocity[0]**2+drone.velocity[1]**2+drone.velocity[2]**2)>0.1):
            print('{} - Sending air break command once again.'.format(time.ctime()))
            drone.send_mavlink(msg)
            print('{} - Body Frame Velocity command is sent! Vx={}, Vy={}, Vz={}'.format(time.ctime(), drone.velocity[0], drone.velocity[1], drone.velocity[2]))
            time.sleep(1)
            # get_vehicle_state(drone)
            print('\n')
    else:
        print('{} - Vehicle is not armed, no need to break.'.format(time.ctime()))

#===================================================

'''
# This message has to be sent again and again.
def fly_follow_leader_onCommand(args_tuple_str):
    global status_waitForCommand
    global leader_host
    
    # Block other commands.
    status_waitForCommand = False
    
    # Get parameters. All original parameters are in string type.
    frame = str(args_tuple_str[0].strip()) # Relative to leader's body frame, or to local frame.
    height = float(args_tuple_str[1].strip()) # Follower's altitude relative to home location.
    radius_2D = float(args_tuple_str[2].strip()) # The projected distance between leader and follower on a 2-D plane.
    azimuth = float(args_tuple_str[3].strip()) # In degree. 0=North(Forward), 90=East(Right), 180=South(back), 270=West(Left).
    print('{} - Calling function fly_follow_leader_onCommand(frame={},height={},radius_2D={},azimuth={}).'.format(time.ctime()), frame, height, radius_2D, azimuth)

    if drone.armed:
        # Request leader drone's gps coordinate.
        print('{} - Requesting leader drone\'s gps coordinate...'.format(time.ctime()))
        lat, lon, alt = Client_request_leader_gps(leader_host)
        leader_heading = Client_request_leader_heading(leader_host)
        if (lat != None) and (leader_heading != None):
            print('{} - Leader drone\'s gps coordinate is : lat={}, lon={}, alt={}'.format(time.ctime(), lat, lon, alt))
            if (frame == 'body'):
                # Calculate follower's new location. This location is based on body frame. (0=North, 90=East)
                new_location_gps = new_gps_coord_after_offset_inBodyFrame((lat, lon), radius_2D, leader_heading, azimuth)
                destination = LocationGlobalRelative(new_location_gps[0], new_location_gps[1], alt)
            elif (frame == 'local'):
                # Calculate follower's new location. This location is based on local frame. (0=North, 90=East)
                new_location_gps = new_gps_coord_after_offset_inLocalFrame((lat, lon), radius_2D, azimuth)
                destination = LocationGlobalRelative(new_location_gps[0], new_location_gps[1], alt)
            elif:
                print('{} - Frame string error, neither \'body\' nor \'local\'. fly_follow_leader_onCommand() is not executed!'.format(time.ctime()))
                break
            # Execute fly command.
            drone.simple_goto(destination)
            print('{} - After executing fly_follow_leader_onCommand(), vehicle status is:'.format(time.ctime()))
            get_vehicle_state(vehicle)
        else:
            print('{} - Did not get GPS coordinate or leader heading direction, fly_follow_leader_onCommand() is not executed!'.format(time.ctime()))
    else:
        print('{} - Vehicle is not armed.'.format(time.ctime()))
    # Enable other commands.
    status_waitForCommand = True
'''
#===================================================

# This message has to be sent again and again.
def fly_follow(drone, followee_host, frame, height, radius_2D, azimuth):
    global status_waitForCommand
    print('{} - Calling function fly_follow().'.format(time.ctime()))
    print('     followee_host={}'.format(followee_host))
    print('     frame={}'.format(frame))
    print('     height={}'.format(height))
    print('     radius_2D={}'.format(radius_2D))
    print('     azimuth={}'.format(azimuth))
    if drone.armed:
        # Request followee drone's gps coordinate.
        print('{} - Requesting followee drone\'s gps coordinate...'.format(time.ctime()))
        lat, lon, alt = CLIENT_request_gps(followee_host)
        followee_heading = CLIENT_request_heading_direction(followee_host)
        
        # Calculate leader and followee distance for avoid crashing.
        _d = geodesic((drone.location.global_relative_frame.lat, drone.location.global_relative_frame.lat), (lat, lon)).km
        print('{} - Distance between host and followee:{}'.format(time.ctime(), _d))
        
        # Calculate destination coordinate based on followee's location.
        if (lat is not None) and (followee_heading is not None):
            print('{} - Followee drone\'s gps coordinate is : lat={}, lon={}, alt={}'.format(time.ctime(), lat, lon, alt))
            if (frame == 'body'):
                # Calculate follower's new location. This location is based on followee's body frame. (0=North, 90=East)
                new_location_gps = new_gps_coord_after_offset_inBodyFrame((lat, lon), radius_2D, followee_heading, azimuth)
                # new_location_gps = get_point_at_distance((lat, lon), radius_2D, followee_heading)
                destination = LocationGlobalRelative(new_location_gps[0], new_location_gps[1], alt)
            elif (frame == 'local'):
                # Calculate follower's new location. This location is based on local frame. (0=North, 90=East)
                new_location_gps = new_gps_coord_after_offset_inLocalFrame((lat, lon), radius_2D, azimuth)
                destination = LocationGlobalRelative(new_location_gps[0], new_location_gps[1], alt)
            else:
                print('{} - Frame error, should be \'body\' or \'local\'. fly_follow() is not executed!'.format(time.ctime()))
                return
            # Execute fly command.
            drone.simple_goto(destination)
            print('{} - After executing fly_follow(), vehicle status is:'.format(time.ctime()))
            # get_vehicle_state(drone)
        else:
            print('{} - Cannot get followee\'s GPS coordinate or heading direction, fly_follow() is not executed!'.format(time.ctime()))
    else:
        print('{} - Vehicle is not armed.'.format(time.ctime()))

#===================================================

def takeoff_and_hover(drone, hover_target_altitude):
    print('\n')
    print('{} - Executing takeoff_and_hover().'.format(time.ctime()))
    drone.simple_takeoff(hover_target_altitude) # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing other command, otherwise the command after drone.simple_takeoff will execute immediately.
    while True:
        print('{} - Current Altitude: {} m'.format(time.ctime(), drone.location.global_relative_frame.alt))
        # get_vehicle_state(drone)        
        print('\n')
        #Break and return from function just below target altitude.
        if drone.location.global_relative_frame.alt>=hover_target_altitude*0.95:
            print('{} - Reached target altitude!\n'.format(time.ctime()))
            break
        time.sleep(1)

#===================================================

def return_to_launch(drone):
    # If vehicle mode is RTL, it will return to the launch location automatically.
    print('\n')
    print('{} - Returning home...'.format(time.ctime()))
    print('{} - Changing vehicle mode to RTL...'.format(time.ctime()))
    drone.mode = VehicleMode('RTL')
    time.sleep(3) # Wait one second for mode change.
    print('{} - Current vehicle mode is {}'.format(time.ctime(), drone.mode))
    while drone.armed:
        print('{} - Vehicle is returning, wait for 1 second.'.format(time.ctime()))
        time.sleep(1)
    print('{} - Vehicle has returned home.'.format(time.ctime()))

#===================================================
"""
# get vehicle state
def get_vehicle_state(drone):
    print('{} - Checking current Vehicle Status:'.format(time.ctime()))
    print('     Global Location: lat={}, lon={}, alt(above sea leavel)={}'.format(drone.location.global_frame.lat, drone.location.global_frame.lon, drone.location.global_frame.alt)) # Absolute GPS coordinate. Its lat and lon attributes are populated shortly after GPS becomes available. The alt can take several seconds longer to populate (from the barometer).
    print('     Global Location (relative altitude): lat={}, lon={}, alt(relative)={}'.format(drone.location.global_relative_frame.lat, drone.location.global_relative_frame.lon, drone.location.global_relative_frame.alt)) # GPS coordinate with relative altitude.
    print('     Local Location(NED coordinate): north={}, east={}, down={}'.format(drone.location.local_frame.north, drone.location.local_frame.east, drone.location.local_frame.down)) # North east down (NED), also known as local tangent plane (LTP)
    print('     Attitude(radians): Pitch={}, Yaw={}, Roll={}'.format(drone.attitude.pitch, drone.attitude.yaw, drone.attitude.roll)) # Pitch, Yaw, and Roll.
    print('     Velocity: Vx={}, Vy={}, Vz={}'.format(drone.velocity[0], drone.velocity[1], drone.velocity[2])) #Current velocity as a three element list [ vx, vy, vz ] (in meter/sec).
    print('     GPS Info: fix_type={}, num_sat={}'.format(drone.gps_0.fix_type, drone.gps_0.satellites_visible)) # GPS Info. fix_type: 0-1, no fix; 2, 2D fix; 3, 3D fix. satellites_visible: Number of satellites visible.
    print('     Battery: voltage={}V, current={}A, level={}%'.format(drone.battery.voltage, drone.battery.current, drone.battery.level))
    print('     Sonar distance: {} m'.format(drone.rangefinder.distance))
    print('     Heading: {} (degrees from North)'.format(drone.heading)) # Current heading in degrees(0~360), where North = 0.
    print('     Groundspeed: {} m/s'.format(drone.groundspeed)) # Current groundspeed in metres/second (double).This attribute is settable. The set value is the default target groundspeed when moving the vehicle using simple_goto() (or other position-based movement commands).
    print('     Airspeed: {} m/s'.format(drone.airspeed)) # Current airspeed in metres/second (double).This attribute is settable. The set value is the default target airspeed when moving the vehicle using simple_goto() (or other position-based movement commands).
"""
#===================================================




