import time
import rs_485 as com
from bitstring import BitArray

def int_byte(int_data):
    return (int_data & 0xff)

def ratio(mid,ang,velo):
    if mid == 2 :
        ang *= 8.0
        velo *= 8.0
    elif mid == 3:
        ang *= 6.0
        velo *= 6.0
    elif mid == 4:
        ang *= -18.0
        velo *= 18.0
    else:
        ang *= -1.0
    return int(ang*100),int(velo*100)

def trans(data):
    #for i in data:
        #print(hex(i))
    com.ser.flushOutput()
    com.ser.write(data)
    while (True):
        # print("wait")
        if(len(com.rx) > 0):
            if((com.rx[0] == data[0]) and (com.rx[1] == data[1]) and (com.rx[2] == data[2])):
                break
    time.sleep(0.01)
def reset_error(mid):
    m_data =[]
    m_data.append(0x3e)
    m_data.append(0x9b)
    m_data.append(mid)
    m_data.append(0x00)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    trans(m_data)

def stopmotor(mid):
    m_data =[]
    m_data.append(0x3e)
    m_data.append(0x80)
    m_data.append(mid)
    m_data.append(0x00)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    trans(m_data)

def runInc_speed(mid,ang,velo):
    chsum = 0
    degree,speed = ratio(mid,ang,velo)
    m_data = []
    m_data.append(0x3e)
    m_data.append(0xa8)
    m_data.append(mid)
    m_data.append(0x08)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    for i in range(5,13):
        if i in range(5,9):
            m_data.append(int_byte(degree >> 8*(i-5)))
        if i in range(9,13):
            m_data.append(int_byte(speed >> 8 * (i - 9)))
        chsum += m_data[i]
    m_data.append(int_byte(chsum))
    trans(m_data)

def runMulti_Angle_speed(mid,ang,velo):
    chsum = 0
    degree,speed = ratio(mid,ang,velo)
    if(speed == 0):
        speed = 1
    m_data = []
    m_data.append(0x3e)
    m_data.append(0xa4)
    m_data.append(mid)
    m_data.append(0x0c)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    for i in range(5,17):
        if i in range(5,13):
            m_data.append(int_byte(degree >> 8*(i-5)))
        if i in range(13,17):
            m_data.append(int_byte(speed >> 8 * (i - 13)))
        chsum += m_data[i]
    m_data.append(int_byte(chsum))
    trans(m_data)

def readAngle(mid):
    _result = 0
    m_data = []
    m_data.append(0x3e)
    m_data.append(0x92)
    m_data.append(mid)
    m_data.append(0x00)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    trans(m_data)
    if((com.rx[0] == 0x3e) and (com.rx[1] == 0x92) and (com.rx[2] == mid)):
        for i in range(5,13):
            _result |= com.rx[i] << (8*(i-5))
    s = "{:016b}".format(_result & 0xffffffff)
    _resulttemp = BitArray(bin=s).int
    result = _resulttemp / 100.0
    if(mid == 2):
        result /= 8.0
    elif(mid == 3):
        result /= 6.0
    elif(mid == 4):
        result /= -18.0
        result /= -18.0
    else:
        result *= -1.0
    return  result

def readSpeed(mid):
    result = 0
    m_data = []
    m_data.append(0x3e)
    m_data.append(0x9c)
    m_data.append(mid)
    m_data.append(0x00)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    trans(m_data)
    if((com.rx[0] == 0x3e) and (com.rx[1] == 0x9c) and (com.rx[2] == mid)):
        result = (com.rx[9] << 8) | (com.rx[8])
        if(mid == 2):
            result /= 8
        elif(mid == 3):
            result /= 6
        elif(mid == 4):
            result /= 18
    return  result

def readEncoder(mid):
    # result = 0
    m_data = []
    m_data.append(0x3e)
    m_data.append(0x90)
    m_data.append(mid)
    m_data.append(0x00)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    trans(m_data)
    while (len(com.rx) == 0):
        print('wait')
    # if((com.rx[0] == 0x3e) and (com.rx[1] == 0x90) and (com.rx[2] == mid)):
    #     result = (com.rx[9] << 8) | (com.rx[8])
    # return  result

def writeM0(mid):
    readEncoder(mid)
    m_data = []
    m_data.append(0x3e)
    m_data.append(0x91)
    m_data.append(mid)
    m_data.append(0x02)
    m_data.append(int_byte(m_data[0] + m_data[1] + m_data[2] + m_data[3]))
    m_data.append(com.rx[7])
    m_data.append(com.rx[8])
    m_data.append(int_byte(m_data[5] + m_data[6]))
    trans(m_data)
    stopmotor(mid)
    runInc_speed(mid,0.01,50)