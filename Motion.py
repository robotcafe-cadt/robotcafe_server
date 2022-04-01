import Motor as mm
import Kinematic as ki
import time as ti
import numpy as np

def Quintic(Q0,Qf,tf,t) :
  A0 = Q0
  A1 = 0.0
  A2 = 0.0
  A3 = 10.0*(Qf-Q0)/(tf**3)
  A4 =-15.0*(Qf-Q0)/(tf**4)
  A5 = 6.0*(Qf-Q0)/(tf**5)
  Qtdot = A1 + 2.0*A2*t + 3.0*A3*(t**2) + 4.0*A4*(t**3) + 5.0*A5*(t**4)
  return Qtdot

def motion(_X,_Y,_Z,d1,_alpha,_belta,_gama,_Tf):
  t = 0
  actAng = []
  calVelo = []
  ki.Inv_K(_X,_Y,_Z,d1,_alpha,_belta,_gama)
  for i in range(1,8):
      actAng.insert(i - 1, np.radians(mm.readAngle(i)))
  while (t < _Tf) : # 1 loop = 60ms
    for i in range(0,7):
      calVelo.insert(i,abs(Quintic(actAng[i], ki.calAngle[i], float(_Tf) / 1000.0, float(t) / 1000.0)))
      # calVelo[i] = max(calVelo[i], limite_velo[i][0]);
      # calVelo[i] = min(calVelo[i], limite_velo[i][1]);
      # print(f"velo{i}")
    t0 = round(ti.time()*1000)
    for i in range(1,8):
      mm.runMulti_Angle_speed(i, float(ki.calAngle[i-1]), float(calVelo[i-1]))
    t1 = round(ti.time()*1000)
    t += (t1-t0)
    # print(t1-t0)