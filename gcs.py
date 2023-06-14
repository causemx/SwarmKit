from core.control import LocationGlobalRelative, connect, ConnectionType
from swarm.swarm_core import (
    arm_no_RC, 
    takeoff_and_hover, 
    get_point_at_distance
    )


def main():
    drone = connect(ConnectionType.udp, "127.0.0.1", 14551)
    arm_no_RC(drone)
    takeoff_and_hover(drone, 20)
    
    _lat = drone.location.global_relative_frame.lat
    _lng = drone.location.global_relative_frame.lon
    pointA = get_point_at_distance((_lat, _lng), 0.1, drone.heading)
    drone.simple_goto(LocationGlobalRelative(pointA[0], pointA[1], 20), 1)


if __name__ == "__main__":
    main()