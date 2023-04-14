import sys
from ctl.control import Core, Drone, ConnectionType


def main():
    core = Core()
    core.connect(ConnectionType.udp, "127.0.0.1")
    d = Drone(core)
    d.initialize()

if __name__ == "__main__":
    main()