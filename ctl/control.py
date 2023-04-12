import atexit
import socket
from threading import Thread
import time
import logging
import math
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



class Drone:
    def __init__(self, core) -> None:
        self._logger = logging.getLogger(__name__)
        self.core = core
        self.master = core.master
        self.core.start()
        self._message_listeners = dict()

    def on_message(self, name):
        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator

    def add_message_listener(self, name, fn):
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)


