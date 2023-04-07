import sys
from ctl import control


def main():
    while True:
        try:
            print('Connect to UAV first.')
            msg = input("> ")
            vehicle = control.Vehicle()
            ret = vehicle.connect(control.ConnectionType.udp, msg)
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