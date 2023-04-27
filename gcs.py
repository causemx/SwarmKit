import sys
import builtins
from ctl.control import Core, Drone, ConnectionType


def main():
    core = Core()
    core.connect(ConnectionType.udp, "127.0.0.1")
    drone = Drone(core)
    drone.initialize()
    builtins.drone = drone

if __name__ == "__main__":
    main()