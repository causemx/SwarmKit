import atexit
import socket
from threading import Thread
import time
import logging
from enum import Enum
from pymavlink import mavutil


class ControlError(Exception):
    pass


class ConnectionType(Enum):
    serial = 1
    udp = 2


class Core:

    def __init__(self) -> None:
        self._logger = logging.getLogger(__name__)
        self.master = None
        self._alive = True
        self._accept_input = True
        
        self.loop_listeners = []
        self.message_listeners = []
        
        self.threadin = t = Thread(target=self.mav_thread_in)
        #t.daemon = True

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
                if hb: return master
            except ControlError:
                print("retry...:{}".format(_retry))
                _retry = _retry + 1
                if _retry > retry:
                    print('fail_to_connect_uav')
                    raise ConnectionError
                time.sleep(0.3)

    def onexit(self):
        self._logger.info("onexit")
        self._alive = False
        self.stop_threads()
    

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
                    except socket.error as error:
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

    def stop_threads(self):
        if self.threadin is not None:
            self.threadin.join
            self.threadin = None

    def forward_message(self, fn):
        """
        Decorator for message inputs
        """
        self.message_listeners.append(fn)

    def forwar_loop(self, fn):
        """
        Decorator for event loop
        """
        self.loop_listeners.append(fn)


class Drone:
    def __init__(self, core) -> None:
        self._logger = logging.getLogger(__name__)
        self.core = core
        self.master = core.master
        self.core.start()
        self._message_listeners = dict()
        
        @core.forward_message
        def foo(_, msg):
            self.notify_message_listener(msg.get_type(), msg)

        """
        HEARTBEAT
        """
        self._flightmode = 'AUTO'
        self._armed = False
        self._system_status = None
        self._autopilot_type = None  # PX4, ArduPilot, etc.
        self._vehicle_type = None  # quadcopter, plane, etc.

        def hb_listener(self, name, m):
            # ignore groundstations
            if m.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._armed = (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            # self.notify_attribute_listeners('armed', self.armed, cache=True)
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

        self.add_message_listener('HEARTBEAT', hb_listener) 

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
        """while True:
            if self._flightmode not in [None, 'INITIALISING', 'MAV']:
                break
            time.sleep(0.1)"""

        # Initialize data stream.
        if rate is not None:
            self._master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                      rate, 1)

        self.add_message_listener('HEARTBEAT', self.send_capabilities_request)

        # Ensure initial parameter download has started.
        while True:
            # This fn actually rate limits itself to every 2s.
            # Just retry with persistence to get our first param stream.
            self._master.param_fetch_all()
            time.sleep(0.1)
            if self._params_count > -1:
                break
       
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

    """def on_message(self, name):
         def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)
            return decorator"""

    def add_message_listener(self, name, fn):
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)
