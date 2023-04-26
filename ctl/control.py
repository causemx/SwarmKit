import atexit
import queue
import socket
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


class VehicleMode():
    def __init__(self, name) -> None:
        self.name = name
    
    def __str__(self):
        return "VehicleMode:%s" % self.name

    def __eq__(self, other):
        return self.name == other

    def __ne__(self, other):
        return self.name != other
    

class HasObservers():
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


class Core:

    def __init__(self) -> None:
        self._logger = logging.getLogger(__name__)
        self.master = None
        self._alive = True
        self._accept_input = True
        
        self.loop_listeners = []
        self.message_listeners = []

        self.out_queue = Queue()
        
        self.threadin = Thread(target=self.mav_thread_in)
        #self.threadin.daemon = True

        self.threadout = Thread(target=self.mav_thread_out)

        atexit.register(self.onexit)

    def connect(self, conn_type, host, port=14550, baud=921600, retry=3):
        _retry = 0
        while True:
            try:
                # master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
                # master = mavutil.mavlink_connection('udpin:{}:{}'.format(host, port))
                if conn_type.value == 1:  # serial type
                    self.master = master = mavutil.mavlink_connection(
                        host, baud=baud)
                elif conn_type.value == 2:  # udp tpye
                    self.master = master = mavutil.mavlink_connection(
                        'udpin:{}:{}'.format(host, port))
                else:
                    return 'unknown_connection_type'

                hb = master.wait_heartbeat()
                if hb: 
                    return master
            except ConnectException:
                print("retry...:{}".format(_retry))
                _retry = _retry + 1
                if _retry > retry:
                    print('fail_to_connect_uav')
                time.sleep(0.3)

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
    def __init__(self, core) -> None:
        super(Drone, self).__init__()

        self._logger = logging.getLogger(__name__)
        self.core = core
        self._master = core.master
        
        self._message_listeners = {}
        self._attribute_listeners = {}
        
        @core.forward_message
        def foo(_, msg):
            self.notify_message_listener(msg.get_type(), msg)

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


        # Deal with Heartbeats
        self._heartbeat_started = False
        self._heartbeat_lastsent = 0
        self._heartbeat_lastreceived = 0
        self._heartbeat_timeout = False

        self._heartbeat_warning = 5
        self._heartbeat_error = 30
        self._heartbeat_system = None

        @core.forward_loop
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

        @core.forward_loop
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
        self.core.start()

        start = time.monotonic()
        self._heartbeat_error = heartbeat_timeout or 0
        self._heartbeat_started = True
        self._heartbeat_lastreceived = start
        

        # Poll for first heartbeat.
        # If heartbeat times out, this will interrupt.
        while self.core._alive:
            time.sleep(.1)
            if self._heartbeat_lastreceived != start:
                break
        if not self.core._alive:
            raise Exception('Timeout in initializing connection.')

        # Register target_system now.
        self.core.target_system = self._heartbeat_system

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

        # Ensure initial parameter download has started.
        while True:
            # This fn actually rate limits itself to every 2s.
            # Just retry with persistence to get our first param stream.
            self._master.param_fetch_all()
            time.sleep(0.1)
            if self._params_count > -1:
                break

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

    @property
    def message_factory(self):
        return self._master.mav

    @property
    def system_status(self):
        # TODO simplized status
        return self._system_status

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
    def armed(self):
        return self._armed

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

    