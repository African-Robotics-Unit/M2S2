from mmWaveClass import mmWaveSystem
radar = mmWaveSystem()
print("Sending Command...")
radar.dca_socket.sendto(radar.dca_cmd['RESET_FPGA_CMD_CODE'],radar.dca_cmd_addr)
try:
    print("Waiting for Command Response...")
    data = radar.dca_socket.recvfrom(2048)
    print("Response: " +str(data))
except Exception as e:
    print("Exception: " + str(e))
