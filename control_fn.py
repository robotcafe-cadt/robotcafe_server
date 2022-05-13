import time
import Motor as motor
import Motion as motion
import Kinematic as kin
import Arduino_port as ardcom
import json

class Payload(object):
    def __init__(self, j):
        self.__dict__ = json.loads(j)

def mainprog():
    for i in range(1, 8):
        motor.reset_error(i)
    for i in range(1, 8):
        motor.runInc_speed(i, 0.1, 90)
    while True:
        time.sleep(1)
        if(len(ardcom.rx) > 0):
            str = ardcom.rx.decode('UTF-8')
            print(str)
            ardcom.rx = bytes(0)
            if(str.index('hM0') > -1):
                motor.stopmotor(1)
                motor.runInc_speed(1,-3600,360)
def run_motion(_x, _y, _z, _alp, _bel, _gam):
    get_x = _x.get()
    get_y = _y.get()
    get_z = _z.get()
    get_alp = _alp.get()
    get_bel = _bel.get()
    get_gam = _gam.get()
    kin.Inv_K(float(get_x), float(get_y), float(get_z), 0.0, float(get_alp), float(get_bel), float(get_gam))
    # motion.motion(float(get_x), float(get_y), float(get_z), 0.0, float(get_alp), float(get_bel), float(get_gam),
    #               10000)

def runInc(value,id, angle, speed):
    get_id = id.get()
    get_angle = angle.get()
    get_speed = speed.get()
    if (value == 1):
        if (get_id == '1'):
            motor.runInc_speed(int(get_id), -int(get_angle), int(get_speed))
        else:
            motor.runInc_speed(int(get_id), int(get_angle), int(get_speed))
    else:
        if (get_id == '1'):
            motor.runInc_speed(int(get_id), int(get_angle), int(get_speed))
        else:
            motor.runInc_speed(int(get_id), -int(get_angle), int(get_speed))

def stop_motor():
    for i in range(1, 8):
        motor.stopmotor(i)
        motor.runInc_speed(i, 0.1, 90)

def run_to0():
    # get_id = id.get()
    # motor.runMulti_Angle_speed(int(get_id),0,90)
    for i in range(1, 8):
        motor.runMulti_Angle_speed(i, 0, 90)