from core.control import LocationGlobalRelative, connect, ConnectionType
from swarm.swarm_core import (
    arm_no_RC, 
    takeoff_and_hover, 
    new_gps_coord_after_offset_inBodyFrame
    )


def main():
    drone = connect(ConnectionType.udp, "127.0.0.1", 14551)
    arm_no_RC(drone)
    takeoff_and_hover(drone, 20)
    
    _lat = drone.location.global_relative_frame.lat
    _lng = drone.location.global_relative_frame.lon
    pointA = new_gps_coord_after_offset_inBodyFrame((_lat, _lng), 20, drone.heading, 0)
    drone.simple_goto(LocationGlobalRelative(pointA[0], pointA[1], 20), 1)


if __name__ == "__main__":
    main()