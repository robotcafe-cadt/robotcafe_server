import serial

# Open serial port Arduino
ard = serial.Serial()
ard.port = 'COM4' #Port Arduino
ard.baudrate = 115200
ard.timeout = 0.01
ard.setDTR(False)
ard.open()
#############################################################

rx = bytes(0)
def read_from_port(ss):
    while True:
        while ss.in_waiting > 0:
            global  rx
            rx = ss.read(100)
            ss.flushInput()
            # print(*rx)
            # rx = bytes(0)