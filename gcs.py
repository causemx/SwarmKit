import argparse
from ctl import control



def main(args):

    parser = argparse.ArgumentParser()
    subparser = parser.add_subparsers()

    parser_connect = subparser.add_parser("connect", help="connect to UAV")
    parser_connect.add_argument("ci", nargs='+', \
        help="For serial port connect, input: 'serial' [device] [baudrate];\
            For UDP input connect, input: 'udp' [host:port].") 
    parser_connect.set_defaults(func=control.connect)

    parser_arm = subparser.add_parser("arm", help="Arming/Disarm UAV throttle.")
    # parser_arm.add_argument('--isarm', type=int, default=1, help="1: arm, 0: disarm.")
    parser_arm.set_defaults(func=control.arm)

    parser_takeoff = subparser.add_parser("takeoff", \
        help="Takeoff UAV to expect height.")
    parser_takeoff.add_argument('height', type=int, default=1,\
                                 help="Expect height for UAV takeoff.")
    parser_takeoff.set_defaults(func=control.takeoff)

    parser_mode = subparser.add_parser("mode", help="Get current UAV mode.")
    parser_mode.set_defaults(func=control.mode)

    parser_setmode = subparser.add_parser("setmode", help="Set UAV mode.")
    parser_setmode.add_argument("mode", type=str, help="Enter expected mode here.")
    parser_setmode.set_defaults(func=control.set_mode)
    
    parser_move = subparser.add_parser("move", 
        help="Make UAV moving [North/South, East/West, Down/Up] in local coord.")
    parser_move.add_argument("movement", type=int, nargs='+', 
        help="Move [east/west] [north/south] [down/up] for meters")
    parser_move.set_defaults(func=control.move)

    args = parser.parse_args(args)
    args.func(args)
   

if __name__ == "__main__":
    import sys
    while True:
        try:
            msg = input("> ")
            main(msg.split())
        except ConnectionError:
            print("connection error")
            sys.exit(1)
        except KeyboardInterrupt:
            print("\n shutdown")
            sys.exit(1) 
                