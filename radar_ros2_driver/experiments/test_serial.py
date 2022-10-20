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