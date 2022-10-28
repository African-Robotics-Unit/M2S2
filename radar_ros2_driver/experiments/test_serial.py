# loop back test
import serial
import time
ser = serial.Serial(port="/dev/ttyACM0",baudrate = 9600, timeout=0.5)
#ser.write(b'dfeDataOutputMode 1\n')
while True:
    time.sleep(1)
    ser.write(b'dfeDataOutputMode 1\n')
    response = ser.read_until(expected=b'\n')#response = ser.read(20)
    if len(response)>0:
        print(response)
    #    ser.write(response)

exit()

# radar serial test
from mmWaveClass import mmWaveSystem
radar = mmWaveSystem()
radar.configure_mmWaveDevice(statusBytes=0)
exit()



# radar serial test 2
import serial
iwr_serial = serial.Serial(port="/dev/hidraw0", 
                            baudrate=115200, 
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE, 
                            stopbits=serial.STOPBITS_ONE, 
                            timeout=2)

cmdBytes =  b"\x64\x66\x65\x44\x61\x74\x61\x4f\x75\x74\x70\x75\x74\x4D\x6f\x64\x65\x20\x31\x0a" #\x0a
iwr_serial.flush()
iwr_serial.write(cmdBytes)
response = iwr_serial.read(len(cmdBytes)+18)
iwr_serial.reset_input_buffer()
print(response)
exit()

# ftdi loop back test
import serial
ser = serial.Serial(port="/dev/ttyUSB0",baudrate = 115200, timeout=0.5, 
                                        bytesize=serial.EIGHTBITS,
                                        parity=serial.PARITY_NONE, 
                                        stopbits=serial.STOPBITS_ONE,)
ser.write(b"Echo Received!!!")
response = ser.read(16)
print(response)
exit()

# echo test
import serial

ser = serial.Serial(port="/dev/ttyUSB0",
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE, 
                    stopbits=serial.STOPBITS_ONE, 
                    timeout=2)
ser.write(b"dfeOutputMode 1\n")
response = ser.read(50)
print(response)
exit()