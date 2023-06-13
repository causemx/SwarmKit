import atexit
import math
import socket
import struct
from threading import Thread
import time
import logging
from enum import Enum
from queue import Queue, Empty
from pymavlink import mavutil


class ControlException(Exception):
    '''Base exception for lib usage.'''

class ConnectException(ControlException):
    '''Raised when connect error.'''

class ConnectionType(Enum):
    serial = 1
    udp = 2


class VehicleMode:
    def __init__(self, name) -> None:
        self.name = name
    
    def __str__(self):
        return "VehicleMode:%s" % self.name

    def __eq__(self, other):
        return self.name == other

    def __ne__(self, other):
        return self.name != other

class GPSInfo():
    """
    Standard information about GPS.

    If there is no GPS lock the parameters are set to ``None``.

    :param Int eph: GPS horizontal dilution of position (HDOP).
    :param Int epv: GPS vertical dilution of position (VDOP).
    :param Int fix_type: 0-1: no fix, 2: 2D fix, 3: 3D fix
    :param Int satellites_visible: Number of satellites visible.

    .. todo:: FIXME: GPSInfo class - possibly normalize eph/epv?  report fix type as string?
    """

    def __init__(self, eph, epv, fix_type, satellites_visible):
        self.eph = eph
        self.epv = epv
        self.fix_type = fix_type
        self.satellites_visible = satellites_visible

    def __str__(self):
        return "GPSInfo:fix=%s,num_sat=%s" % (self.fix_type, self.satellites_visible)

class Battery():
    """
    System battery information.

    An object of this type is returned by :py:attr:`Vehicle.battery`.

    :param voltage: Battery voltage in millivolts.
    :param current: Battery current, in 10 * milliamperes. ``None`` if the autopilot does not support current measurement.
    :param level: Remaining battery energy. ``None`` if the autopilot cannot estimate the remaining battery.
    """

    def __init__(self, voltage, current, level):
        self.voltage = voltage / 1000.0
        if current == -1:
            self.current = None
        else:
            self.current = current / 100.0
        if level == -1:
            self.level = None
        else:
            self.level = level

    def __str__(self):
        return f"Battery:voltage={self.voltage},current={self.current},level={self.level}"

class SystemStatus(object):
    """
    This object is used to get and set the current "system status".

    An object of this type is returned by :py:attr:`Vehicle.system_status`.

    .. py:attribute:: state

        The system state, as a ``string``.
    """

    def __init__(self, state):
        self.state = state

    def __str__(self):
        return f"SystemStatus:{self.state}"

    def __eq__(self, other):
        return self.state == other

    def __ne__(self, other):
        return self.state != other

class Attitude(object):
    """
    Attitude information.

    An object of this type is returned by :py:attr:`Vehicle.attitude`.

    .. _figure_attitude:

    .. figure:: http://upload.wikimedia.org/wikipedia/commons/thumb/c/c1/Yaw_Axis_Corrected.svg/500px-Yaw_Axis_Corrected.svg.png
        :width: 400px
        :alt: Diagram showing Pitch, Roll, Yaw
        :target: http://commons.wikimedia.org/wiki/File:Yaw_Axis_Corrected.svg

        Diagram showing Pitch, Roll, Yaw (`Creative Commons <http://commons.wikimedia.org/wiki/File:Yaw_Axis_Corrected.svg>`_)

    :param pitch: Pitch in radians
    :param yaw: Yaw in radians
    :param roll: Roll in radians
    """

    def __init__(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

    def __str__(self):
        fmt = '{}:pitch={pitch},yaw={yaw},roll={roll}'
        return fmt.format(self.__class__.__name__, **vars(self))

class HasObservers:
    """
    Implementation of observer pattern for add/remove/notify listener.
    """
    def __init__(self) -> None:
        logging.basicConfig()
        self._logger = logging.getLogger(__name__)
        self._attribute_listeners = {}
        self._attribute_cache = {}

    def add_attribute_listener(self, attr_name, observer):
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is None:
            listeners_for_attr = []
            self._attribute_listeners[attr_name] = listeners_for_attr
        if observer not in listeners_for_attr:
            listeners_for_attr.append(observer)

    def remove_attribute_listener(self, attr_name, observer):
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is not None:
            listeners_for_attr.remove(observer)
            if len(listeners_for_attr) == 0:
                del self._attribute_listeners[attr_name]

    def notify_attribute_listeners(self, attr_name, value, cache=False):   
        if cache:
            if self._attribute_cache.get(attr_name) == value:
                return
            self._attribute_cache[attr_name] = value

        # Notify observers.
        for fn in self._attribute_listeners.get(attr_name, []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

        for fn in self._attribute_listeners.get('*', []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

    def on_attribute(self, name):
         def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_attribute_listener(n, fn)
            else:
                self.add_attribute_listener(name, fn)

            return decorator

class Parameters(HasObservers):
    def __init__(self, drone) -> None:
        super(Parameters, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._drone = drone

    def set(self, name, value, retries=3):
        name = name.upper()
        # convert to single precision floating point number (the type used by low level mavlink messages)
        value = float(struct.unpack('f', struct.pack('f', value))[0])
        remaining = retries
        while True:
            self._drone._master.param_set_send(name, value)
            tstart = time.monotonic()
            if remaining == 0:
                break
            remaining -= 1
            while time.monotonic() - tstart < 1:
                if name in self._drone._params_map and self._drone._params_map[name] == value:
                    return True
                time.sleep(0.1)
    
    def get(self, name):
        name = name.upper()
        return self._drone._params_map.get(name)
    
    
    def add_attribute_listener(self, attr_name, *args, **kwargs):
        attr_name = attr_name.upper()
        return super(Parameters, self).add_attribute_listener(attr_name, *args, **kwargs)
        # return super().add_attribute_listener(attr_name, observer)

    def remove_attribute_listener(self, attr_name, *args, **kwargs):
        attr_name = attr_name.upper()
        return super(Parameters, self).remove_attribute_listener(attr_name, *args, **kwargs)
    
    def notify_attribute_listeners(self, attr_name, *args, **kwargs):
        attr_name = attr_name.upper()
        return super().notify_attribute_listeners(attr_name, *args, **kwargs)
    
    def on_attribute(self, attr_name, *args, **kwargs):
        attr_name = attr_name.upper()
        return super().on_attribute(attr_name, *args, **kwargs)
    
class Locations(HasObservers):
    """
    An object for holding location information in global, global relative and local frames.

    :py:class:`Vehicle` owns an object of this type. See :py:attr:`Vehicle.location` for information on
    reading and observing location in the different frames.

    The different frames are accessed through the members, which are created with this object.
    They can be read, and are observable.
    """

    def __init__(self, drone):
        super(Locations, self).__init__()

        self._lat = None
        self._lon = None
        self._alt = None
        self._relative_alt = None

        @drone.on_message('GLOBAL_POSITION_INT')
        def listener(drone, name, m):
            (self._lat, self._lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self._relative_alt = m.relative_alt / 1000.0
            self.notify_attribute_listeners('global_relative_frame', self.global_relative_frame)
            drone.notify_attribute_listeners('location.global_relative_frame',
                                               drone.location.global_relative_frame)

            if self._alt is not None or m.alt != 0:
                # Require first alt value to be non-0
                # TODO is this the proper check to do?
                self._alt = m.alt / 1000.0
                self.notify_attribute_listeners('global_frame', self.global_frame)
                drone.notify_attribute_listeners('location.global_frame',
                                                   drone.location.global_frame)

            drone.notify_attribute_listeners('location', drone.location)

        self._north = None
        self._east = None
        self._down = None

        @drone.on_message('LOCAL_POSITION_NED')
        def listener(drone, name, m):
            self._north = m.x
            self._east = m.y
            self._down = m.z
            self.notify_attribute_listeners('local_frame', self.local_frame)
            drone.notify_attribute_listeners('location.local_frame', drone.location.local_frame)
            drone.notify_attribute_listeners('location', drone.location)

    @property
    def local_frame(self):
        """
        Location in local NED frame (a :py:class:`LocationGlobalRelative`).

        This is accessed through the :py:attr:`Vehicle.location` attribute:

        .. code-block:: python

            print "Local Location: %s" % vehicle.location.local_frame

        This location will not start to update until the vehicle is armed.
        """
        return LocationLocal(self._north, self._east, self._down)

    @property
    def global_frame(self):
        """
        Location in global frame (a :py:class:`LocationGlobal`).

        The latitude and longitude are relative to the
        `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
        The altitude is relative to mean sea-level (MSL).

        This is accessed through the :py:attr:`Vehicle.location` attribute:

        .. code-block:: python

            print "Global Location: %s" % vehicle.location.global_frame
            print "Sea level altitude is: %s" % vehicle.location.global_frame.alt

        Its ``lat`` and ``lon`` attributes are populated shortly after GPS becomes available.
        The ``alt`` can take several seconds longer to populate (from the barometer).
        Listeners are not notified of changes to this attribute until it has fully populated.

        To watch for changes you can use :py:func:`Vehicle.on_attribute` decorator or
        :py:func:`add_attribute_listener` (decorator approach shown below):

        .. code-block:: python

            @vehicle.on_attribute('location.global_frame')
            def listener(self, attr_name, value):
                print " Global: %s" % value

            #Alternatively, use decorator: ``@vehicle.location.on_attribute('global_frame')``.
        """
        return LocationGlobal(self._lat, self._lon, self._alt)

    @property
    def global_relative_frame(self):
        """
        Location in global frame, with altitude relative to the home location
        (a :py:class:`LocationGlobalRelative`).

        The latitude and longitude are relative to the
        `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
        The altitude is relative to :py:attr:`home location <Vehicle.home_location>`.

        This is accessed through the :py:attr:`Vehicle.location` attribute:

        .. code-block:: python

            print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
            print "Altitude relative to home_location: %s" % vehicle.location.global_relative_frame.alt
        """
        return LocationGlobalRelative(self._lat, self._lon, self._relative_alt)

class LocationLocal:
    """
    A local location object.

    The north, east and down are relative to the EKF origin.  This is most likely the location where the vehicle was turned on.

    An object of this type is owned by :py:attr:`Vehicle.location`. See that class for information on
    reading and observing location in the local frame.

    :param north: Position north of the EKF origin in meters.
    :param east: Position east of the EKF origin in meters.
    :param down: Position down from the EKF origin in meters. (i.e. negative altitude in meters)
    """

    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down

    def __str__(self):
        return f"LocationLocal:north={self.north},east={self.east},down={self.down}"

    def distance_home(self):
        """
        Distance away from home, in meters. Returns 3D distance if `down` is known, otherwise 2D distance.
        """

        if self.north is not None and self.east is not None:
            if self.down is not None:
                return math.sqrt(self.north**2 + self.east**2 + self.down**2)
            else:
                return math.sqrt(self.north**2 + self.east**2)

class LocationGlobal:
    """
    A global location object.

    The latitude and longitude are relative to the `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
    The altitude is relative to mean sea-level (MSL).

    For example, a global location object with altitude 30 metres above sea level might be defined as:

    .. code:: python

       LocationGlobal(-34.364114, 149.166022, 30)

    .. todo:: FIXME: Location class - possibly add a vector3 representation.

    An object of this type is owned by :py:attr:`Vehicle.location`. See that class for information on
    reading and observing location in the global frame.

    :param lat: Latitude.
    :param lon: Longitude.
    :param alt: Altitude in meters relative to mean sea-level (MSL).
    """

    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # This is for backward compatibility.
        self.local_frame = None
        self.global_frame = None

    def __str__(self):
        return f"LocationGlobal:lat={self.lat},lon={self.lon},alt={self.alt}"

class LocationGlobalRelative:
    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # This is for backward compatibility.
        self.local_frame = None
        self.global_frame = None

    def __str__(self):
        return f"LocationGlobalRelative:lat={self.lat},lon={self.lon,},alt={self.alt}"

class Handler:

    def __init__(self, master) -> None:
        self._logger = logging.getLogger(__name__)
        self.master = master
        self._alive = True
        self._accept_input = True
        
        self.loop_listeners = []
        self.message_listeners = []

        self.out_queue = Queue()
        
        self.threadin = Thread(target=self.mav_thread_in)
        self.threadin.daemon = True

        self.threadout = Thread(target=self.mav_thread_out)
        self.threadout.daemon = True

        atexit.register(self.onexit)

    def onexit(self):
        self._logger.info("onexit")
        self._alive = False
        self.stop_threads()

    def mav_thread_out(self):
            # Huge try catch in case we see http://bugs.python.org/issue1856
            try:
                while self._alive:
                    try:
                        msg = self.out_queue.get(True, timeout=0.01)
                        self.master.write(msg)
                    except Empty:
                        continue
                    except socket.error as error:
                        # If connection reset (closed), stop polling.
                        if error.errno == -1:
                            raise ControlException('Connection aborting during read')
                        raise
                    except Exception as e:
                        self._logger.exception('mav send error: %s' % str(e))
                        break
            except ControlException as e:
                self._logger.exception("Exception in MAVLink write loop", exc_info=True)
                self._alive = False
                self.master.close()
                self._death_error = e

            except Exception as e:
                # http://bugs.python.org/issue1856
                if not self._alive:
                    pass
                else:
                    self._alive = False
                    self.master.close()
                    self._death_error = e

            # Explicitly clear out buffer so .close closes.
            self.out_queue = Queue()

    def mav_thread_in(self):
        try:
            while self._alive:
                # Loop listeners.
                for fn in self.loop_listeners:
                    fn(self)

                # Sleep
                self.master.select(0.05)

                while self._accept_input:
                    try:
                        msg = self.master.recv_msg()
                    except socket.error:
                        # If connection reset (closed), stop polling.
                        msg = None
                    except mavutil.mavlink.MAVError as e:
                        # Avoid
                        #   invalid MAVLink prefix '73'
                        #   invalid MAVLink prefix '13'
                        self._logger.debug('mav recv error: %s' % str(e))
                        msg = None
                    except Exception:
                        # Log any other unexpected exception
                        self._logger.exception(
                            'Exception while receiving message: ', exc_info=True)
                        msg = None
                    if not msg:
                        break

                    # Message listeners.
                    for fn in self.message_listeners:
                        try:
                            fn(self, msg)
                        except Exception:
                            self._logger.exception(
                                'Exception in message handler for %s' % msg.get_type(), exc_info=True
                            )
        except Exception:
            self._logger.exception('Exception in MAVLink input loop')
            self._alive = False
            self.master.close()
            return
    
    def start(self):
        if not self.threadin.is_alive():
            self.threadin.start()
        if not self.threadout.is_alive():
            self.threadout.start()


    def close(self):
        # TODO this can block forever if parameters continue to be added
        self._alive = False
        while not self.out_queue.empty():
            time.sleep(0.1)
        self.stop_threads()
        self.master.close()

    def stop_threads(self):
        if self.threadin is not None:
            self.threadin.join()
            self.threadin = None
        if self.threadout is not None:
            self.threadout.join()
            self.threadout = None
        

    def forward_message(self, fn):
        '''Decorator for message inputs'''
        self.message_listeners.append(fn)

    def forward_loop(self, fn):
        '''Decorator for event loop'''
        self.loop_listeners.append(fn)


class Drone(HasObservers):
    _instance = None
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, handler) -> None:
        super(Drone, self).__init__()
        self._logger = logging.getLogger(__name__)
        self.handler = handler
        self._master = handler.master
        
        self._message_listeners = {}
        self._attribute_listeners = {}
        
        @handler.forward_message
        def foo(_, msg):
            self.notify_message_listener(msg.get_type(), msg)


        self._location = Locations(self)

        self._vx = None
        self._vy = None
        self._vz = None


        self._wind_direction = None
        self._wind_speed = None
        self._wind_speed_z = None

        @self.on_message('WIND')
        def listener(self,name, m):
            """ WIND {direction : -180.0, speed : 0.0, speed_z : 0.0} """
            self._wind_direction = m.direction
            self._wind_speed = m.speed
            self._wind_speed_z = m.speed_z

        @self.on_message('GLOBAL_POSITION_INT')
        def listener(self, name, m):
            (self._vx, self._vy, self._vz) = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
            self.notify_attribute_listeners('velocity', self.velocity)


        # Deal with heartbeat
        self._flightmode = 'AUTO'
        self._armed = False
        self._system_status = None
        self._autopilot_type = None  # PX4, ArduPilot, etc.
        self._vehicle_type = None  # quadcopter, plane, etc.

        @self.on_message('HEARTBEAT')
        def hb_listener(self, name, m):
            # ignore groundstations
            if m.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._armed = (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            #self.notify_attribute_listeners('armed', self.armed, cache=True)
            self._autopilot_type = m.autopilot
            self._vehicle_type = m.type
            if self._is_mode_available(m.custom_mode, m.base_mode) is False:
                raise Exception("mode (%s, %s) not available on mavlink definition" % (m.custom_mode, m.base_mode))
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self._flightmode = mavutil.interpret_px4_mode(m.base_mode, m.custom_mode)
            else:
                self._flightmode = self._mode_mapping_bynumber[m.custom_mode]
            self.notify_attribute_listeners('mode', self.mode, cache=True)
            self._system_status = m.system_status
            self.notify_attribute_listeners('system_status', self.system_status, cache=True)


        self._pitch = None
        self._yaw = None
        self._roll = None
        self._pitchspeed = None
        self._yawspeed = None
        self._rollspeed = None

        @self.on_message('ATTITUDE')
        def listener(self, name, m):
            self._pitch = m.pitch
            self._yaw = m.yaw
            self._roll = m.roll
            self._pitchspeed = m.pitchspeed
            self._yawspeed = m.yawspeed
            self._rollspeed = m.rollspeed
            self.notify_attribute_listeners('attitude', self.attitude)

        self._heading = None
        self._airspeed = None
        self._groundspeed = None

        @self.on_message('VFR_HUD')
        def listener(self, name, m):
            self._heading = m.heading
            self.notify_attribute_listeners('heading', self.heading)
            self._airspeed = m.airspeed
            self.notify_attribute_listeners('airspeed', self.airspeed)
            self._groundspeed = m.groundspeed
            self.notify_attribute_listeners('groundspeed', self.groundspeed)
        
        self._voltage = None
        self._current = None
        self._level = None

        @self.on_message('SYS_STATUS')
        def listener(self, name, m):
            self._voltage = m.voltage_battery
            self._current = m.current_battery
            self._level = m.battery_remaining
            self.notify_attribute_listeners('battery', self.battery)

        self._eph = None
        self._epv = None
        self._satellites_visible = None
        self._fix_type = None  # FIXME support multiple GPSs per vehicle - possibly by using componentId

        @self.on_message('GPS_RAW_INT')
        def listener(self, name, m):
            self._eph = m.eph
            self._epv = m.epv
            self._satellites_visible = m.satellites_visible
            self._fix_type = m.fix_type
            self.notify_attribute_listeners('gps_0', self.gps_0)
            
            

        
        # Deal with parameters

        start_duration = 0.2
        repeat_duration = 1

        self._params_count = -1
        self._params_count = -1
        self._params_set = []
        self._params_loaded = False
        self._params_start = False
        self._params_map = {}
        self._params_last = time.monotonic()  # Last new param.
        self._params_duration = start_duration
        self._parameters = Parameters(self)

        @handler.forward_loop
        def listener(_):
            # Check the time duration for last "new" params exceeds watchdog.
            if not self._params_start:
                return

            if not self._params_loaded and all(x is not None for x in self._params_set):
                self._params_loaded = True
                self.notify_attribute_listeners('parameters', self.parameters)

            if not self._params_loaded and time.monotonic() - self._params_last > self._params_duration:
                c = 0
                for i, v in enumerate(self._params_set):
                    if v is None:
                        self._master.mav.param_request_read_send(0, 0, b'', i)
                        c += 1
                        if c > 50:
                            break
                self._params_duration = repeat_duration
                self._params_last = time.monotonic()

        @self.on_message(['PARAM_VALUE'])
        def listener(self, name, msg):
            # If we discover a new param count, assume we
            # are receiving a new param set.
            if self._params_count != msg.param_count:
                self._params_loaded = False
                self._params_start = True
                self._params_count = msg.param_count
                self._params_set = [None] * msg.param_count

            # Attempt to set the params. We throw an error
            # if the index is out of range of the count or
            # we lack a param_id.
            try:
                if msg.param_index < msg.param_count and msg:
                    if self._params_set[msg.param_index] is None:
                        self._params_last = time.monotonic()
                        self._params_duration = start_duration
                    self._params_set[msg.param_index] = msg

                self._params_map[msg.param_id] = msg.param_value
                self._parameters.notify_attribute_listeners(msg.param_id, msg.param_value,
                                                            cache=True)
            except:
                import traceback
                traceback.print_exc()

        # Deal with Heartbeats
        self._heartbeat_started = False
        self._heartbeat_lastsent = 0
        self._heartbeat_lastreceived = 0
        self._heartbeat_timeout = False

        self._heartbeat_warning = 5
        self._heartbeat_error = 30
        self._heartbeat_system = None

        @handler.forward_loop
        def listener(_):
            # Send 1 heartbeat per second
            if time.monotonic() - self._heartbeat_lastsent > 1:
                self._master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self._heartbeat_lastsent = time.monotonic()

            # Timeouts.
            if self._heartbeat_started:
                if self._heartbeat_error and time.monotonic() - self._heartbeat_lastreceived > self._heartbeat_error > 0:
                    raise Exception('No heartbeat in %s seconds, aborting.' %
                                       self._heartbeat_error)
                elif time.monotonic() - self._heartbeat_lastreceived > self._heartbeat_warning:
                    if self._heartbeat_timeout is False:
                        self._logger.warning('Link timeout, no heartbeat in last %s seconds' % self._heartbeat_warning)
                        self._heartbeat_timeout = True

        @self.on_message(['HEARTBEAT'])
        def listener(self, name, msg):
            # ignore groundstations
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._heartbeat_system = msg.get_srcSystem()
            self._heartbeat_lastreceived = time.monotonic()
            if self._heartbeat_timeout:
                self._logger.info('...link restored.')
            self._heartbeat_timeout = False

        self._last_heartbeat = None

        @handler.forward_loop
        def listener(_):
            if self._heartbeat_lastreceived:
                self._last_heartbeat = time.monotonic() - self._heartbeat_lastreceived
                self.notify_attribute_listeners('last_heartbeat', self.last_heartbeat)


    def on_message(self, name):
        def decorator(func):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, func)
            else:
                self.add_message_listener(name, func)
        return decorator

    def initialize(self, rate=4, heartbeat_timeout=30):
        self.handler.start()

        start = time.monotonic()
        self._heartbeat_error = heartbeat_timeout or 0
        self._heartbeat_started = True
        self._heartbeat_lastreceived = start
        

        # Poll for first heartbeat.
        # If heartbeat times out, this will interrupt.
        while self.handler._alive:
            time.sleep(.1)
            if self._heartbeat_lastreceived != start:
                break
        if not self.handler._alive:
            raise Exception('Timeout in initializing connection.')

        # Register target_system now.
        self.handler.target_system = self._heartbeat_system

        # Wait until board has booted.
        while True:
            if self._flightmode not in [None, 'INITIALISING', 'MAV']:
                break
            time.sleep(0.1)

        # Initialize data stream.
        if rate is not None:
            self._master.mav.request_data_stream_send(\
                0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)

        self.add_message_listener('HEARTBEAT', self.send_capabilities_request)

        self._master.mav.param_request_list_send(
            self._master.target_system, self._master.target_component
        )

        # Ensure initial parameter download has started.
        # TODO Fetch params when starting.
        """while True:
            # This fn actually rate limits itself to every 2s.
            # Just retry with persistence to get our first param stream.
            self._master.param_fetch_all()
            time.sleep(0.1)
            if self._params_count > -1:
                break"""

    def send_capabilities_request(self, drone, name, m):
        '''Request an AUTOPILOT_VERSION packet'''
        capability_msg = drone.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)
        drone.send_mavlink(capability_msg)

    def notify_message_listener(self, name, msg):
        for fn in self._message_listeners.get(name, []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception(f"Exception in message handler for {msg.get_type()}", exc_info=True)

        for fn in self._message_listeners.get('*', []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception(f"Exception in message handler for {msg.get_type()}", exc_info=True)

    def add_message_listener(self, name, fn):
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def send_mavlink(self, message):
        '''Send custom message to the drone'''
        self._master.mav.send(message)
        
    def simple_takeoff(self, alt=None):
        """
        Take off and fly the vehicle to the specified altitude (in metres) and then wait for another command.

        .. note::

            This function should only be used on Copter vehicles.


        The vehicle must be in GUIDED mode and armed before this is called.

        There is no mechanism for notification when the correct altitude is reached,
        and if another command arrives before that point (e.g. :py:func:`simple_goto`) it will be run instead.

        .. warning::

           Apps should code to ensure that the vehicle will reach a safe altitude before
           other commands are executed. A good example is provided in the guide topic :doc:`guide/taking_off`.

        :param alt: Target height, in metres.
        """
        if alt is not None:
            altitude = float(alt)
            if math.isnan(altitude) or math.isinf(altitude):
                raise ValueError("Altitude was NaN or Infinity. Please provide a real number")
            self._master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, altitude)

    def simple_goto(self, location, airspeed=None, groundspeed=None):
        '''
        Go to a specified global location (:py:class:`LocationGlobal` or :py:class:`LocationGlobalRelative`).

        There is no mechanism for notification when the target location is reached, and if another command arrives
        before that point that will be executed immediately.

        You can optionally set the desired airspeed or groundspeed (this is identical to setting
        :py:attr:`airspeed` or :py:attr:`groundspeed`). The vehicle will determine what speed to
        use if the values are not set or if they are both set.

        The method will change the :py:class:`VehicleMode` to ``GUIDED`` if necessary.

        .. code:: python

            # Set mode to guided - this is optional as the simple_goto method will change the mode if needed.
            vehicle.mode = VehicleMode("GUIDED")

            # Set the LocationGlobal to head towards
            a_location = LocationGlobal(-34.364114, 149.166022, 30)
            vehicle.simple_goto(a_location)

        :param location: The target location (:py:class:`LocationGlobal` or :py:class:`LocationGlobalRelative`).
        :param airspeed: Target airspeed in m/s (optional).
        :param groundspeed: Target groundspeed in m/s (optional).
        '''
        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            alt = location.alt
        elif isinstance(location, LocationGlobal):
            # This should be the proper code:
            # frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            # However, APM discards information about the relative frame
            # and treats any alt value as relative. So we compensate here.
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            if not self.home_location:
                self.commands.download()
                self.commands.wait_ready()
            alt = location.alt - self.home_location.alt
        else:
            raise ValueError('Expecting location to be LocationGlobal or LocationGlobalRelative.')

        self._master.mav.mission_item_send(0, 0, 0, frame,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, 0, location.lat, location.lon,
                                           alt)

        if airspeed is not None:
            self.airspeed = airspeed
        if groundspeed is not None:
            self.groundspeed = groundspeed

    @property
    def message_factory(self):
        return self._master.mav

    @property
    def last_heartbeat(self):
        return self._last_heartbeat

    @property
    def _mode_mapping(self):
        return self._master.mode_mapping()

    @property
    def _mode_mapping_bynumber(self):
        return mavutil.mode_mapping_bynumber(self._vehicle_type)

    def _is_mode_available(self, custommode_code, basemode_code=0):
        try:
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                mode = mavutil.interpret_px4_mode(basemode_code, custommode_code)
                return mode in self._mode_mapping
            return custommode_code in self._mode_mapping_bynumber
        except ControlException:
            return False

    @property
    def gps_0(self):
        """
        GPS position information (:py:class:`GPSInfo`).
        """
        return GPSInfo(self._eph, self._epv, self._fix_type, self._satellites_visible)
    
    @property
    def armed(self):
        return self._armed

    @armed.setter
    def armed(self, value):
        if bool(value) is not self._armed:
            if value:
                self._master.arducopter_arm()
            else:
                self._master.arducopter_disarm()
                
    @property
    def is_armable(self):
        """
        Returns ``True`` if the vehicle is ready to arm, false otherwise (``Boolean``).

        This attribute wraps a number of pre-arm checks, ensuring that the vehicle has booted,
        has a good GPS fix, and that the EKF pre-arm is complete.
        """
        #* check that mode is not INITIALSING
        #* check that we have a GPS fix
        #* check that EKF pre-arm is complete
        # TODO Maybe something change belong version upgrade.
        return self.mode != 'INITIALISING' and \
            (self.gps_0.fix_type is not None and self.gps_0.fix_type > 1) #and self._ekf_predposhorizabs

    @property
    def mode(self):
        """
        This attribute is used to get and set the current flight mode. You
        can specify the value as a :py:class:`VehicleMode`, like this:

        .. code-block:: python

           vehicle.mode = VehicleMode('LOITER')

        Or as a simple string:

        .. code-block:: python

            vehicle.mode = 'LOITER'

        If you are targeting ArduPilot you can also specify the flight mode
        using a numeric value (this will not work with PX4 autopilots):

        .. code-block:: python

            # set mode to LOITER
            vehicle.mode = 5
        """
        if not self._flightmode:
            return None
        return VehicleMode(self._flightmode)

    @mode.setter
    def mode(self, v):
        if isinstance(v, str):
            v = VehicleMode(v)

        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self._master.set_mode(v.name)
        elif isinstance(v, int):
            self._master.set_mode(v)
        else:
            self._master.set_mode(self._mode_mapping[v.name])

    @property
    def system_status(self):
        """
        System status (:py:class:`SystemStatus`).

        The status has a ``state`` property with one of the following values:

        * ``UNINIT``: Uninitialized system, state is unknown.
        * ``BOOT``: System is booting up.
        * ``CALIBRATING``: System is calibrating and not flight-ready.
        * ``STANDBY``: System is grounded and on standby. It can be launched any time.
        * ``ACTIVE``: System is active and might be already airborne. Motors are engaged.
        * ``CRITICAL``: System is in a non-normal flight mode. It can however still navigate.
        * ``EMERGENCY``: System is in a non-normal flight mode. It lost control over parts
          or over the whole airframe. It is in mayday and going down.
        * ``POWEROFF``: System just initialized its power-down sequence, will shut down now.
        """
        return {
            0: SystemStatus('UNINIT'),
            1: SystemStatus('BOOT'),
            2: SystemStatus('CALIBRATING'),
            3: SystemStatus('STANDBY'),
            4: SystemStatus('ACTIVE'),
            5: SystemStatus('CRITICAL'),
            6: SystemStatus('EMERGENCY'),
            7: SystemStatus('POWEROFF'),
        }.get(self._system_status, None)

    @property
    def battery(self):
        """
        Current system batter status (:py:class:`Battery`).
        """
        if self._voltage is None or self._current is None or self._level is None:
            return None
        return Battery(self._voltage, self._current, self._level)


    @property
    def groundspeed(self):
        """
        Current groundspeed in metres/second (``double``).

        This attribute is settable. The set value is the default target groundspeed
        when moving the vehicle using :py:func:`simple_goto` (or other position-based
        movement commands).
        """
        return self._groundspeed

    @property
    def heading(self):
        """
        Current heading in degrees - 0..360, where North = 0 (``int``).
        """
        return self._heading
    
    @property
    def attitude(self):
        """
        Current vehicle attitude - pitch, yaw, roll (:py:class:`Attitude`).
        """
        return Attitude(self._pitch, self._yaw, self._roll)
    
    @property
    def airspeed(self):
        """
        Current airspeed in metres/second (``double``).

        This attribute is settable. The set value is the default target airspeed
        when moving the vehicle using :py:func:`simple_goto` (or other position-based
        movement commands).
        """
        return self._airspeed
    
    @property
    def velocity(self):
        """
        Current velocity as a three element list ``[ vx, vy, vz ]`` (in meter/sec).
        """
        return [self._vx, self._vy, self._vz]
    
    @property
    def location(self):
        """
        The vehicle location in global, global relative and local frames (:py:class:`Locations`).

        The different frames are accessed through its members:

        * :py:attr:`global_frame <dronekit.Locations.global_frame>` (:py:class:`LocationGlobal`)
        * :py:attr:`global_relative_frame <dronekit.Locations.global_relative_frame>` (:py:class:`LocationGlobalRelative`)
        * :py:attr:`local_frame <dronekit.Locations.local_frame>` (:py:class:`LocationLocal`)

        For example, to print the location in each frame for a ``vehicle``:

        .. code-block:: python

            # Print location information for `vehicle` in all frames (default printer)
            print "Global Location: %s" % vehicle.location.global_frame
            print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
            print "Local Location: %s" % vehicle.location.local_frame    #NED

            # Print altitudes in the different frames (see class definitions for other available information)
            print "Altitude (global frame): %s" % vehicle.location.global_frame.alt
            print "Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt
            print "Altitude (NED frame): %s" % vehicle.location.local_frame.down

        .. note::

            All the location "values" (e.g. ``global_frame.lat``) are initially
            created with value ``None``. The ``global_frame``, ``global_relative_frame``
            latitude and longitude values are populated shortly after initialisation but
            ``global_frame.alt`` may take a few seconds longer to be updated.
            The ``local_frame`` does not populate until the vehicle is armed.

        The attribute and its members are observable. To watch for changes in all frames using a listener
        created using a decorator (you can also define a listener and explicitly add it).

        .. code-block:: python

            @vehicle.on_attribute('location')
            def listener(self, attr_name, value):
                # `self`: :py:class:`Vehicle` object that has been updated.
                # `attr_name`: name of the observed attribute - 'location'
                # `value` is the updated attribute value (a :py:class:`Locations`). This can be queried for the frame information
                print " Global: %s" % value.global_frame
                print " GlobalRelative: %s" % value.global_relative_frame
                print " Local: %s" % value.local_frame

        To watch for changes in just one attribute (in this case ``global_frame``):

        .. code-block:: python

            @vehicle.on_attribute('location.global_frame')
            def listener(self, attr_name, value):
                # `self`: :py:class:`Locations` object that has been updated.
                # `attr_name`: name of the observed attribute - 'global_frame'
                # `value` is the updated attribute value.
                print " Global: %s" % value

            #Or watch using decorator: ``@vehicle.location.on_attribute('global_frame')``.
        """
        return self._location
    
    @property
    def parameters(self):
        """
        The (editable) parameters for this vehicle (:py:class:`Parameters`).
        """
        return self._parameters
    
def connect(conn_type, host, port=14550, baud=921600, retry=3):
    _retry = 0
    while True:
        try:
            # master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
            # master = mavutil.mavlink_connection('udpin:{}:{}'.format(host, port))
            if conn_type.value == 1:  # serial type
                master = mavutil.mavlink_connection(host, baud=baud)
            elif conn_type.value == 2:  # udp tpye
                master = mavutil.mavlink_connection('udpin:{}:{}'.format(host, port))
            else:
                return 'unknown_connection_type'

            hb = master.wait_heartbeat()
            if hb:
                _handler = Handler(master)
                _drone = Drone(_handler)
                _drone.initialize()
                return _drone
        except ConnectException:
            print("retry...:{}".format(_retry))
            _retry = _retry + 1
            if _retry > retry:
                print('fail_to_connect_uav')
            time.sleep(0.3)