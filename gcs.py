import sys
import time
import signal
from core.control import connect, ConnectionType
from swarm.swarm_core import (
    arm_no_RC, 
    takeoff_and_hover, 
    )

def signal_handler(signum, frame):
    print('signal_handler: caught signal ' + str(signum))
    if signum == signal.SIGINT.value:
        print('SIGINT')
        sys.exit(1)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    print(signal.SIGINT)
    
    drone = connect(ConnectionType.udp, "127.0.0.1", 14551)
    # arm_no_RC(drone)
    # takeoff_and_hover(drone, 20)
    while True:
        time.sleep(1)
    
    
    '''
    _lat = drone.location.global_relative_frame.lat
    _lng = drone.location.global_relative_frame.lon
    pointA = get_point_at_distance((_lat, _lng), 0.1, drone.heading)
    drone.simple_goto(LocationGlobalRelative(pointA[0], pointA[1], 20), 1)
    '''

if __name__ == "__main__":
    main()