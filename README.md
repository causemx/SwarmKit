# SwarmKit

This is the wrapper for drone control and swarming library.

The wrapper is based on pymavlink and server/client communicating written in python-socket. To use the wrapper server called "swarm" function to running on the same system by multiple threading.


## Important Notes

- Python 3.7+ is required (threading will be duprecated, instead of [asyncio](https://docs.python.org/3.7/library/asyncio.html)).
- You may need to run `pip3` instead of `pip` and `python3` instead of `python`, depending of your system defaults.


## Installation
first, build up a virtual enviroment
```sh
python3 -m venv [YOUR_ENV]
```
and then run:

```sh
source [YOUR_ENV]/bin/activate
```
install requirments:

```sh
pip3 install -r requirments.txt
```

The package contains `mavsdk_server` already (previously called "backend"), which is started automatically when connecting (e.g. `await drone.connect()`). Have a look at the examples to see it used in practice. It will be something like:

```python
from ctl.control import Core, Drone, ConnectionType

...

core = Core()
# you can choose type for udp or serial_port
core.connect(ConnectionType.udp, [PORT])
drone = Drone(core)
drone.initialize()
```

Note: `core.connect --help` will show detail information for uav connection.

## Run the examples

Once the package has been installed, the examples can be run:

```sh
python3 ./gcs.py
```
