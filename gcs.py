import time
from core.control import connect, ConnectionType
from swarm.swarm_core import (
    arm_no_RC, 
    takeoff_and_hover, 
    )


def main():
    drone = connect(ConnectionType.udp, "127.0.0.1", 14551)
    #arm_no_RC(drone)
    #takeoff_and_hover(drone, 20)
    
    tstart = time.monotonic()
    while time.monotonic() - tstart < 100:
        print("time going") 
        time.sleep(1)
    
    '''
    _lat = drone.location.global_relative_frame.lat
    _lng = drone.location.global_relative_frame.lon
    pointA = get_point_at_distance((_lat, _lng), 0.1, drone.heading)
    drone.simple_goto(LocationGlobalRelative(pointA[0], pointA[1], 20), 1)
    '''

if __name__ == "__main__":
    main()