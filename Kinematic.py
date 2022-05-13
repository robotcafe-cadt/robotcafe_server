import math
import numpy as np
# import Motor as mm

limite_ang = [[ -110.0, 110.0], [ -90.0, 90.0], [ -36.0, 90.0], [ -180.0, 180.0], [ -120.0, 120.0], [ -180.0, 180.0]]
limite_velo = [[ 0, 90], [ 0, 90], [ 0, 90], [0, 90], [ 0, 90], [ 0, 90]]
calVelo = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
calAngle = []

a1 = 70.0
a2 = 223.0
a3 = 224.97
a4 = 117.8

theta0= theta1= theta2= theta3= theta4=theta5= theta6= lamda= phi= 0.0
cas = 0
dpr = 2.0/360.0 #Motor1 distance of arm move per revolution

def Inv_K(_X,_Y,_Z,d1,_alpha,_belta,_gama):
    global theta0, theta1, theta2, theta3, theta4, theta5, theta6, lamda, phi, pdr
    alpha = np.radians(_alpha)
    belta = np.radians(_belta)
    gama = np.radians(_gama)
    X = _X - a4 * math.cos(belta) * math.cos(alpha)
    Y = _Y - a4 * math.cos(belta) * math.sin(alpha)
    Z = _Z - a4 * math.sin(belta)
    if((X != 0.0) or (Y != 0.0)):
        theta0 = d1 / dpr
        theta1 = math.atan2(Y, X - d1)
        A = (X - d1) * math.cos(theta1) + (Y * math.sin(theta1))
        B = Z - a1
        C = ((a3**2) - (A**2) - (B**2) - (a2**2)) / (2 * a2)
        phi = math.atan2(B , A)
        # phi = math.atan(B/A)
        R = math.sqrt((A**2) + (B**2))
        theta2 = math.asin(C / R) + phi
        lamda = math.atan2((B - (a2 * math.cos(theta2))), (A + (a2 * math.sin(theta2))))
        theta3 = lamda - theta2

        #End-Effector Rotation
        R0_3 = [[-math.cos(theta1) * math.sin(theta2 + theta3), math.sin(theta1), math.cos(theta1) * math.cos(theta2 + theta3)],
                     [-math.sin(theta1) * math.sin(theta2 + theta3), -math.cos(theta1), math.sin(theta1) * math.cos(theta2 + theta3)],
                     [math.cos(theta2 + theta3), 0.0, math.sin(theta2 + theta3)]
                     ]
        InvR0_3 = np.linalg.inv(R0_3)
        Vec = [[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]]
        Euler_XYZ = [[math.cos(belta) * math.cos(gama), -math.cos(belta) * math.sin(gama), math.sin(belta)],
                     [math.sin(gama) * math.cos(alpha) + math.sin(alpha) * math.sin(belta) * math.cos(gama),
                      math.cos(alpha) * math.cos(gama) - math.sin(alpha) * math.sin(belta) * math.sin(gama), -math.sin(alpha) * math.cos(belta)],
                     [math.sin(alpha) * math.sin(gama) - math.cos(alpha) * math.cos(gama) * math.sin(belta),
                      math.cos(gama) * math.sin(alpha) + math.cos(alpha) * math.sin(belta) * math.sin(gama), math.cos(alpha) * math.cos(belta)]]
        Vec_R0_6 = np.dot(Vec, Euler_XYZ)
        R3_6 = np.dot(InvR0_3, Vec_R0_6)
        check = False
        global cas
        cas = 0
        while (check == False):
            if(cas == 0):
                theta5 = math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0):
                    theta4 = math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = math.atan2(-R3_6[2][1], R3_6[2][0])
                else:
                    theta4 = math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            if (cas == 1):
                theta5 = math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0) :
                    theta4 = math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = -math.atan2(-R3_6[2][1], R3_6[2][0])
                else:
                    theta4 = math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = -math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            if (cas == 2):
                theta5 = math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0) :
                    theta4 = -math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = math.atan2(-R3_6[2][1], R3_6[2][0])
                else :
                    theta4 = -math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            if (cas == 3):
                theta5 = math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0) :
                    theta4 = -math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = -math.atan2(-R3_6[2][1], R3_6[2][0])
                else :
                    theta4 = -math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = -math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            if (cas == 4):
                theta5 = -math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0) :
                    theta4 = math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = math.atan2(-R3_6[2][1], R3_6[2][0])
                else :
                    theta4 = math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            if (cas == 5):
                theta5 = -math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0) :
                    theta4 = math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = -math.atan2(-R3_6[2][1], R3_6[2][0])
                else :
                    theta4 = math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = -math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            if (cas == 6):
                theta5 = -math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0) :
                    theta4 = -math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = math.atan2(-R3_6[2][1], R3_6[2][0])
                else :
                    theta4 = -math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            if (cas == 7):
                theta5 = -math.acos(checkangle_result(R3_6[2][2]))
                if (theta5 == 0.0) :
                    theta4 = -math.atan2(R3_6[1][2], R3_6[0][2])
                    theta6 = -math.atan2(-R3_6[2][1], R3_6[2][0])
                else :
                    theta4 = -math.acos(checkangle_result(R3_6[0][2] / math.sin(theta5)))
                    theta6 = -math.acos(checkangle_result(-R3_6[2][0] / math.sin(theta5)))
            CheckR3_6 = [[(math.cos(theta4) * math.cos(theta5) * math.cos(theta6)) - (math.sin(theta4) * math.sin(theta6)),
                          (-math.cos(theta4) * math.cos(theta5) * math.sin(theta6)) - (math.sin(theta4) * math.cos(theta6)),
                          math.cos(theta4) * math.sin(theta5)],
                         [(math.sin(theta4) * math.cos(theta5) * math.cos(theta6)) + (math.cos(theta4) * math.sin(theta6)),
                          (-math.sin(theta4) * math.cos(theta5) * math.sin(theta6)) + (math.cos(theta4) * math.cos(theta6)),
                          math.sin(theta4) * math.sin(theta5)],
                         [-math.sin(theta5) * math.cos(theta6), math.sin(theta5) * math.sin(theta6), math.cos(theta5)]
                         ]
            CheckR0_6 = np.dot(R0_3, CheckR3_6)
            # print(f"case{cas}")
            # # print(CheckR0_6)
            # print(np.degrees(theta4))
            # print(np.degrees(theta5))
            # print(np.degrees(theta6))

            check = check_Rotation(CheckR0_6, Vec_R0_6)

        #Join Angle
        calAngle.insert(0,theta0)
        calAngle.insert(1,np.degrees(theta1))
        calAngle.insert(2,np.degrees(theta2))
        calAngle.insert(3,np.degrees(theta3))
        calAngle.insert(4,np.degrees(theta4))
        calAngle.insert(5,np.degrees(theta5))
        calAngle.insert(6,np.degrees(theta6))
        for i in range(0,7):
            print(f"theta{i} : {calAngle[i]}")

def checkangle_result(input):
    # deta = abs(1 - abs(input))
    # if (deta < 1E-4) :
    #     if (input > 0) :
    #         result = 1.0
    #     else :
    #         result = -1.0
    # else :
    result = input
    return result

def check_Rotation(MatrixCheck, MatrixResult):
    global cas
    numcorrect = 0
    submatrix = np.subtract(MatrixResult, MatrixCheck)
    for i in range(0,3):
        for j in range(0,3):
            if((submatrix[j][i] <= 2.0E-2) and (submatrix[j][i] >= -2.0E-2)):
                numcorrect += 1
    if (numcorrect == 9) :
        if ((np.degrees(theta4) == 180.0) or (np.degrees(theta4) == -180.0)) :
            result = False
            cas += 1
        else :
            print(f"solution case {cas}")
            result = True
    else :
        result = False
        cas += 1
    if (cas > 7) :
        print("no solution")
        result = True
    return result

def Fwd_K():
    global dpr
    actAng = []
    # for i in range(1,8):
    #     actAng.insert(i-1,np.radians(mm.readAngle(i)))
    _theta0 = actAng[0]
    _theta1 = actAng[1]
    _theta2 = actAng[2]
    _theta3 = actAng[3]
    _theta4 = actAng[4]
    _theta5 = actAng[5]
    _theta6 = actAng[6]

    _d1 = _theta0 * dpr
    H0_1 = [[math.cos(_theta1), 0.0, math.sin(_theta1), _d1],
            [math.sin(_theta1), 0.0, -math.cos(_theta1), 0.0],
            [0.0, 1.0, 0.0, a1],
            [0.0, 0.0, 0.0, 1.0]]
    H1_2 = [[-math.sin(_theta2), -math.cos(_theta2), 0.0, -a2 * math.sin(_theta2)],
            [math.cos(_theta2), -math.sin(_theta2), 0.0, a2 * math.cos(_theta2)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]
    H2_3 = [[math.cos(_theta3), 0.0, math.sin(_theta3), 0.0],
            [math.sin(_theta3), 0.0, -math.cos(_theta3), 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]
    H3_4 = [[math.cos(_theta4), 0.0, -math.sin(_theta4), 0.0],
            [math.sin(_theta4), 0.0, math.cos(_theta4), 0.0],
            [0.0, -1.0, 0.0, a3],
            [0.0, 0.0, 0.0, 1.0]]
    H4_5 = [[math.cos(_theta5), 0.0, math.sin(_theta5), 0.0],
            [math.sin(_theta5), 0.0, -math.cos(_theta5), 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]
    H5_6 = [[math.cos(_theta6), -math.sin(_theta6), 0.0, 0.0],
            [math.sin(_theta6), math.cos(_theta6), 0.0, 0.0],
            [0.0, 0.0, 1.0, a4],
            [0.0, 0.0, 0.0, 1.0]]
    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3, H3_4)
    H0_5 = np.dot(H0_4, H4_5)
    H0_6 = np.dot(H0_5, H5_6)

    ActX = H0_6[0][3]
    ActY = H0_6[1][3]
    ActZ = H0_6[2][3]
