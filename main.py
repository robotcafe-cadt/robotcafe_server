#import Motor as motor
#import rs_485 as com
import Kinematic as kin
import time
import threading
# testing
if __name__ == '__main__':
    # thread = threading.Thread(target=com.read_from_port, args=(com.ser,))
    # thread.start()
    kin.Inv_K(0.0,-(kin.a3+kin.a4),kin.a1+kin.a2,0.0,0.0,0.0,0.0)
    kin.Inv_K(0.0, kin.a3 + kin.a4, kin.a1 + kin.a2, 0.0, 0.0, 0.0, 0.0)
    while True:
        print("Get Start")
        time.sleep(2)
        # if(len(com.rx) > 0):
        #     print(*com.rx)
        #     com.rx = bytes(0)

        # motor.runMulti_Angle_speed(2,-360.5,360.7)

