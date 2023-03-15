import argparse
import time
from enum import Enum
from threading import Thread
from pymavlink import mavutil


master = None

class ControlError(Exception):
    pass


class ConnectionType(Enum):
    serial = 1
    udp = 2 


def connect(conn_type, host, port=14550, baud=921600, retry=3) -> str:
    global master
    # _args = args.connect_info
    _retry = 0
    while True:
        try:
            #master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
            #master = mavutil.mavlink_connection('udpin:{}:{}'.format(host, port))
            if conn_type.value == 1: # serial type
                master = mavutil.mavlink_connection(host,baud=baud)
            elif conn_type.value == 2: # udp tpye
                master = mavutil.mavlink_connection('udpin:{}:{}'.format(host, port))
            else:
                return 'unknown_connection_type'

            m = master.wait_heartbeat()
            if m:
                Thread(target=handle_input, args=(master,)).start() 

            return 'connected'
        except ControlError:
            print("retry...:{}".format(_retry))
            _retry = _retry + 1
            if _retry > retry:
                print('fail_to_connect_uav')
                raise ConnectionError
            time.sleep(0.3)


def handle_input(master):
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking = True)
        mode = mavutil.mode_string_v10(msg)
        ipt = input("{}> ".format(mode))
        print(ipt)
    


def parse(args):
    parser = argparse.ArgumentParser()
    subparser = parser.add_subparsers()

    parser_arm = subparser.add_parser("arm", help="Arming/Disarm UAV throttle.")
    # parser_arm.add_argument('--isarm', type=int, default=1, help="1: arm, 0: disarm.")
    parser_arm.set_defaults(func=arm)

    parser_takeoff = subparser.add_parser("takeoff", \
        help="Takeoff UAV to expect height.")
    parser_takeoff.add_argument('height', type=int, default=1,\
                                 help="Expect height for UAV takeoff.")
    parser_takeoff.set_defaults(func=takeoff)


    parser_setmode = subparser.add_parser("setmode", help="Set UAV mode.")
    parser_setmode.add_argument("mode", type=str, help="Enter expected mode here.")
    parser_setmode.set_defaults(func=set_mode)
    
    parser_move = subparser.add_parser("move", 
        help="Make UAV moving [North/South, East/West, Down/Up] in local coord.")
    parser_move.add_argument("movement", type=int, nargs='+', 
        help="Move [east/west] [north/south] [down/up] for meters")
    parser_move.set_defaults(func=move)

    args = parser.parse_args(args)
    args.func(args)





def arm(args) -> None:
    master.mav.command_long_send(master.target_system, master.target_component, \
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, args.isarm, 0, 0, 0, 0, 0, 0)

    while True:
        master.motors_armed_wait()
        print('Ack!')
        break


def takeoff(args) -> None:
    master.mav.command_long_send(master.target_system, master.target_component, \
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, args.height)

    _retry = 0
    while True:
        _retry = _retry + 1
        # msg = master.recv_match(type='STATUSTEXT', blocking=True)
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if msg: 
            print(msg)
            break
        if _retry > 3:
            print("Time out, can not recive ACK.")
            break


def mode(args) -> None:
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking = True)
        mode = mavutil.mode_string_v10(msg)
        print(mode)
        break

def set_mode(args) -> None:
    if args.mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(args.mode))
        print('Try:', list(master.mode_mapping().keys()))
        pass # in application could be sys.exit(1)

        # Get mode ID
    mode_id = master.mode_mapping()[args.mode]

    master.mav.set_mode_send(master.target_system, \
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    while True:
        try:
            ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        except ControlError:
            print("can not receive ack")
            break
        
        time.sleep(0.3)
        ack_msg = ack_msg.to_dict()

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

def move(args) -> None:
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, master.target_system, master.target_component, \
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), \
                args.movement[0], args.movement[1], args.movement[2], \
                    0, 0, 0, 0, 0, 0, 0, 0))
    
    while True:
        msg = master.recv_match(
            type='LOCAL_POSITION_NED', blocking=True)
        print(msg)
        break

