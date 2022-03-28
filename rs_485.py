import serial

# Open serial port
ser = serial.Serial()
ser.port = 'COM5'
ser.baudrate = 115200
ser.timeout = 0.01
ser.setDTR(False)
ser.open()
#############################################################
rx = bytes(0)
def read_from_port(ss):
    while True:
        while ss.in_waiting > 0:
            global  rx
            rx = ss.read(100)
            ss.flushInput()