import sys
from ctl import control
from ctl.control import ConnectionType as ct


def main():
    while True:
        try:
            print('Connect to UAV fist.')
            msg = input("> ")
            ret = control.connect(ct.udp, msg)
            if ret:
                print(ret)
                return 0
        except ConnectionError:
            print("connection error")
            return 1
        except KeyboardInterrupt:
            print("\n shutdown")
            return 1 
                
       

if __name__ == "__main__":
    error_code = main()
    if error_code == 1:
        sys.exit(1)