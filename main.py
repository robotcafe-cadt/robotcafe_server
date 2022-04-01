import Motor as motor
import rs_485 as com
import Kinematic as kin
import time
import threading
import Motion as motion
import tkinter as tk


def runIncMp():
    motor.runInc_speed(2,1,90)
def runIncMm():
    motor.runInc_speed(2,-1,90)
def mainprog():
    # motor.runInc_speed(3, 1, 90)
    # kin.Inv_K(0.0,-(kin.a3+kin.a4),(kin.a1+kin.a2),0.0,0.0,0.0,0.0)
    # time.sleep(2)
    # print(f"angle={motor.readAngle(3)}")

    # for i in range(1,8):
    #     motor.runInc_speed(i, 0.1, 90)
    # for i in range(1,8):
    #     motor.runMulti_Angle_speed(i, 0, 90)

    # time.sleep(5)
    # motion.motion(0.0,(kin.a3+kin.a4),(kin.a1+kin.a2),0.0,0.0,0.0,0.0,10000)
    # time.sleep(12)
    # motion.motion(0.0, -(kin.a3 + kin.a4), (kin.a1 + kin.a2), 0.0, 0.0, 0.0, 0.0, 10000)
    while True:
        # print("hi")
        time.sleep(1)
        # if(len(com.rx) > 0):
        #     print(*com.rx)
        #     com.rx = bytes(0)

        # motor.runMulti_Angle_speed(2,-360.5,360.7)

if __name__ == '__main__':
    thread1 = threading.Thread(target=com.read_from_port, args=(com.ser,))
    thread1.start()
    thread2 = threading.Thread(target=mainprog)
    thread2.start()

    win = tk.Tk()
    win.geometry("340x128")
    btn1 = tk.Button(win, text="UP", command=runIncMp)
    btn1.place(x=0,y=0)
    btn2 = tk.Button(win, text="DOWN", command=runIncMm)
    btn2.pack()

    win.mainloop()



