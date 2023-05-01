from ctl.control import Core, Drone, ConnectionType


def main():
    core = Core()
    core.connect(ConnectionType.udp, "127.0.0.1")
    drone = Drone(core)
    drone.initialize()

    print(drone.armed)
    print(drone.mode)
    print(drone.system_status)
    print(drone.location.global_frame)


if __name__ == "__main__":
    main()