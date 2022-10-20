# radar serial test 2
import serial
iwr_serial = serial.Serial(port="/dev/ttyACM2", 
                            baudrate=115200, 
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE, 
                            stopbits=serial.STOPBITS_ONE, 
                            timeout=0.5)
iwr_serial.flush()
iwr_serial.write(b"dfeDataOutputMode 1\r\n")
response = iwr_serial.read(len(b"dfeDataOutputMode 1\r\n")+18)
iwr_serial.reset_input_buffer()
print(response)
exit()

# radar serial test
from mmWaveClass import mmWaveSystem
radar = mmWaveSystem()
radar.configure_mmWaveDevice(statusBytes=20)
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