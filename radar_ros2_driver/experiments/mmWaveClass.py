# %---------------------------------------------------------------------------------------------------------
# Standard Python Libraries
import socket
import time
import struct
import os

# %---------------------------------------------------------------------------------------------------------
# PySerial Library (needs to be installed)
import serial 

# %---------------------------------------------------------------------------------------------------------
# Custom Modules (same directory as this script)
from mmWaveConfigClass import mmWaveConfig
from circularBufferClass import frameBuffer

class mmWaveSystem():

    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    # useful commands
    iwr_rec_cmd = ['sensorStop', 'sensorStart'] 
    dca_cmd = { \
        'RESET_FPGA_CMD_CODE'               : b"\x5a\xa5\x01\x00\x00\x00\xaa\xee", \
        'CONFIG_FPGA_GEN_CMD_CODE'          : b"\x5a\xa5\x03\x00\x06\x00\x01\x02\x01\x02\x03\x1e\xaa\xee", \
        'CONFIG_PACKET_DATA_CMD_CODE'       : b"\x5a\xa5\x0b\x00\x06\x00\xbe\x05\x35\x0c\x00\x00\xaa\xee", \
        'SYSTEM_CONNECT_CMD_CODE'           : b"\x5a\xa5\x09\x00\x00\x00\xaa\xee", \
        'RECORD_START_CMD_CODE'             : b"\x5a\xa5\x05\x00\x00\x00\xaa\xee", \
        'SYSTEM_ERROR_CMD_CODE'             : b"\x5a\xa5\x0a\x00\x01\x00\xaa\xee", \
        'RECORD_STOP_CMD_CODE'              : b"\x5a\xa5\x06\x00\x00\x00\xaa\xee", \
    }   # commands sent over ethernet based on CLI. Consists of header, data size, command code, footer
        # commands above are listed in order of send and receive
        # some commands in the DCA1000 user guide are not used and are thus not included

    # Sockets and Ports
    dca_cmd_addr = ('192.168.33.180', 4096)     # address to send commands to
    dca_res_addr = ('192.168.33.30', 4096)      # address to receive command responses from
    dca_data_addr =("192.168.33.30", 4098)      # address to receive radar data from
    dca_socket = None                           # socket variable for the command port
    data_socket = None                          # socket variable for the data stream port
    iwr_serial = None                           # UART com port to configure iwr device

    # configs
    runTimeDict = None
    configDict = None

    # flags
    DCA1000_error_flag = None
    mmWaveDevice_error_flag = None
    
    # system buffer
    frameSize = None    # size in bytes
    buf = None

    # file writing
    fileCounter = None
    filePerFrame = None
    filePath = None
    fileName = None
    bytesWritten = None
    frameCount = None

    # %---------------------------------------------------------------------------------------------------------
    # Class Functions
    # %---------------------------------------------------------------------------------------------------------
    # Class constructor
    def __init__(self, runtimeConfigPath = None, mmwaveConfigPath = None, rcvBufMult = 2):
        """Initializes the radar sensor System and creates radar object.

    This function creates the radar object. It creates the config object that loads
    the config parameters from the configuration given as arguments. Runtime config file
    deals with COM Port selection, and file writing and formatting options and details which
    board is being used. The mmWaveconfig file stores all the device parameters such as the chirp
    profile and is parsed to the radar board via the Command COM port specified in the runtime config. 
    Function also inits the Command COM port and Etherent socket for mmWave Device and DCA1000 
    respectively. It also creates a new folder for adc data to be stored in and initializes the buffer.  

    :param string runtimeConfigPath: file path to runtime config file. (Default = None)
    :param string mmwaveConfigPath: file path to mmwave config file. (Default = None)
    :param int rcvBufMult: multiplier for receive buffer size for data socket. Increase if experiencing packet loss. (Default = 2)
    :raises AnyError: raises erros and exits program execution if etherent socket or com port inits failed
    """
        # print CLI header
        print("")
        print("%------------------- mmWave Device Raw ADC Data Collection Started ----------------------%")
        print("")

        # create mmWaveConfig object to store config params
        # if config paths set to None, default values stored in conig class will be loaded
        self.configs = mmWaveConfig(runtimeConfigPath,mmwaveConfigPath)
        self.runTimeDict = self.configs.runtimeParamDict
        self.configDict = self.configs.mmwaveParamDict

        print("")
        print("%----------------------- Initializing Sockets and Serial Ports --------------------------%")
        # create socket for data port (all radar data traffic is collected from here)
        try:
            self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.data_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,65536*rcvBufMult)
            self.data_socket.bind(self.dca_data_addr)
            self.data_socket.settimeout(25e-5)
            self.data_socket_open = True 
            print(":> Data Socket initialized")
        except Exception as e:
            print(":> Error-> Data Socket init failed")
            print("\tException: " + str(e)) 

        # create socket for command port (all radar commands and responses sent and collected here)
        try:
            self.dca_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.dca_socket.bind(self.dca_res_addr)
            self.dca_socket.settimeout(2)
            self.dca_socket_open = True
            print(":> Command Socket initialized")
        except Exception as e:
            print(":> Error-> Command Socket init failed")
            print("\tException: " + str(e)) 
        
        # setup serial comms 
        try: 
            self.iwr_serial = serial.Serial(port="/dev/ttyUSB0", 
                                        baudrate=115200, 
                                        bytesize=serial.EIGHTBITS,
                                        parity=serial.PARITY_NONE, 
                                        stopbits=serial.STOPBITS_ONE, 
                                        timeout=0.7)
            print(":> COM Port initialized")
        except Exception as e:
            print(":> Error-> UART Config Port init failed")
            print("\tException: " + str(e)) 
        self.captureStarted = False

        print("")
        # Create subdirectory for adc_data storage
        #folderCount = 0
        #mkdirSuccess = False
        #while not mkdirSuccess:
        #    try:
        #        os.mkdir(self.runTimeDict["CAPTURED_ADC_DATA_PATH"]+"_"+str(folderCount))
        #        mkdirSuccess=True
        #    except Exception as e:
        #        folderCount+=1 # if folder already present increment suffix by 1. "_0" -> "_1" etc.
        #print("")    
        
        # setup system buffer for data stream
        numChirps = self.configDict["frameCfg"]["numChirps"]
        numSamples = self.configDict["profileCfg"]["numAdcSamples"]
        bytesPerSample = 2 # adc either returns 12,14 or 16bit data which all have to be stored over 2 bytes 
        realOrComplex = (self.configDict["adcbufCfg"]["adcOutputFmt"]+2) % 3 # if 0 it is complex so 2%3 = 2, if 1 it is real 3%3=1
        recBitMask =  bin(self.configDict["channelCfg"]["rxChannelEn"])[2:] # convert number to bit mask string with bin() and remove 0b from start of string  
        numReceivers = 0
        for bit in recBitMask:  # count the number of bits = 1 in bit mask to get number of receiver channels enabled
            numReceivers = numReceivers + int(bit)
        self.frameSize = numChirps*numSamples*bytesPerSample*realOrComplex*numReceivers # calculate required bufferSize
        self.buf = frameBuffer(self.frameSize)

        # setup file write parameters
        #self.fileCounter = 0
        #self.bytesWritten = 0
        #self.frameCount = 0
        #self.packNum = 0
        #self.allFramesCollected = False
        #self.filePath = self.runTimeDict["CAPTURED_ADC_DATA_PATH"]+"_"+str(folderCount)+"/"
        #self.fileName = self.runTimeDict["DCA_FILE_PREFIX"]+"_"
        #if self.runTimeDict["DCA_MAX_REC_FILE_SIZE_MB"]==0:
        #    self.filePerFrame = True # write each frame to separate file
        #else:
        #    self.filePerFrame = False # write file till number of bytes reached 
    
    # Primary Functions
    # %---------------------------------------------------------------------------------------------------------    
    def setup_DCA1000(self):
        """Performs setup of DCA1000 over etherent. 

    Sends commands over etherent to command port of DCA1000. Commands
    are listed in dictionary at start of class definition. 
    Function Can cause program to hang on waiting for successful acknowledge of 
    command from DCA1000. 

    :raises AnyError: raises errors and exits program execution if setup commands failed
    """
        if not self.dca_socket:
            print(":> DCA1000 setup failed: Socket not open")
            print("   Exiting Program")
            print("")
            exit()

        # Set up DCA
        print("%-------------------------- Setting up DCA1000 for Capture ------------------------------%")
        print("")

        # send commands and await response
        self.dca_socket.sendto(self.dca_cmd['RESET_FPGA_CMD_CODE'], self.dca_cmd_addr)
        self.response_DCA1000('RESET_FPGA_CMD_CODE')
        print("")

        self.dca_socket.sendto(self.dca_cmd['CONFIG_FPGA_GEN_CMD_CODE'], self.dca_cmd_addr)
        self.response_DCA1000('CONFIG_FPGA_GEN_CMD_CODE')
        print("")

        self.dca_socket.sendto(self.dca_cmd['CONFIG_PACKET_DATA_CMD_CODE'], self.dca_cmd_addr)
        self.response_DCA1000('CONFIG_PACKET_DATA_CMD_CODE')
        print("")

        self.dca_socket.sendto(self.dca_cmd['SYSTEM_CONNECT_CMD_CODE'], self.dca_cmd_addr)
        self.response_DCA1000('CONFIG_PACKET_DATA_CMD_CODE')
        print("")

        if self.DCA1000_error_flag:
            print(":> DCA1000 setup failed")
            print("   Exiting Program")
            print("")
            exit()
        else:
            print(":> DCA1000 setup succeeded") 
            print("")
        
    # %---------------------------------------------------------------------------------------------------------
    def configure_mmWaveDevice(self, statusBytes = 6):
        """Configures mmWaveDevice over UART. 

    Sends commands and config setting over user UART COM Port of mmWave Device. Commands
    are extracted from a JSON file and stored in dictionary. Device returns will return
    [cmdString given, status, "mmWaveDemo:/>"] as a list. \\n is used as list separator. 
    Thus device reads in bytes equal to length of command string and 6 extra bytes for status. 
    Can increase bytes read after cmdString for info if error command is received. 
    Prints command sent and status received.

    :param int statusBytes: bytes to read in after cmdString received. (Default is 6)
    :raises AnyError: raises errors and exits program execution if error response was received from any commands
    """
        print("%------------------- Loading mmWave Device Configurations to Board ----------------------%")
        if not self.iwr_serial:
            print(":> Error-> Serial Port not open")
            return

        variant = self.runTimeDict["VARIANT"]

        # Set up Radar EVM
        print(":> Configuring {variant} board".format(variant=variant))
        print("")

        self.mmWaveDevice_error_flag = False
        self.iwr_serial.flush()
        # write config file to device 
        for cmdKey in list(self.configDict.keys()):
            cmdString = self.configs.get_mmWaveCommandString(cmdKey) # need this line to remove _x suffix from command before being written to serial
            self.write_Serial(cmdString)
            response = self.iwr_serial.read(len(cmdString))#+statusBytes
            #self.iwr_serial.reset_input_buffer()
            print(':> ' + cmdString.strip('\n'))
            try:
                #print(" Status-> "+response.decode('utf-8').split('\n')[1]) # some error codes can be found under studio_cli\src\mss\mmwave_cli.h
                #if response.decode('utf-8').split('\n')[1] == "Error":
                #    self.mmWaveDevice_error_flag = True
                print("Response: " + str(response))
                print("")
            except:
                self.mmWaveDevice_error_flag = True
                print(" Status-> Error: Invalid response received. Received: {response}".format(response=response))
                print("")
            time.sleep(0.6)

        # exit program if config failed
        if self.mmWaveDevice_error_flag:
            print(":> mmWave Device Config failed")
            print("   Exiting Program")
            exit()
      
    # %---------------------------------------------------------------------------------------------------------
    def start_Record(self):
        """Starts the recording process.  

    First sends the record start command over ethernet to tell the DCA1000 to start monitoring LVDS lanes.
    Then writes the serial command "sensorStart" to the mmWaveDevice to start waveform transmission.
    It is recommended to call this function after starting threads responsible for reading in and writing radar data
    so that PC is already waiting for incomming data. This ensures no delay between starting the sensor and pulling data 
    into script. 

    :raises AnyError: raises error and exits program execution if error response was received from startSensor or startRecord command
    """
        self.dca_socket.sendto(self.dca_cmd['RECORD_START_CMD_CODE'], self.dca_cmd_addr)
        #self.response_DCA1000('RECORD_START_CMD_CODE')
        print("")

        self.write_Serial((self.iwr_rec_cmd[1]+"\n"))
        self.captureStarted = True
        response = self.iwr_serial.read(len(self.iwr_rec_cmd[1])+6)
        self.iwr_serial.reset_input_buffer()
        print(':> ' + self.iwr_rec_cmd[1].strip('\n'))
        print(" Status->\t"+response.decode('utf-8').split('\n')[1]+"\n")
        print("")

        if response.decode('utf-8').split('\n')[1] == "Error":
                self.mmWaveDevice_error_flag = True

        if self.mmWaveDevice_error_flag:
            print(":> mmWave Device Start failed")
            print("   Exiting Program")
            exit()

        if self.DCA1000_error_flag:
            print(":> DCA1000 record start failed")
            print("   Exiting Program")
            exit()    

    # %---------------------------------------------------------------------------------------------------------
    def collect_Data(self):
        """Function call to collect data from ethernet port and add to buffer.  

    Receives UDP packet from the data port. Will exit function if time out error is received. 
    This function only supports the RAW data output format and not seperated data.
    The DCA1000 raw data format is: Packet Number [4 Bytes], Byte ID [6 Bytes], Raw Data [max 1456 Bytes].
    Function first extracts packet number to check for dropped packets and will indicate if packets are dropped.
    
    If packets are dropped function will zero fill missing packets. The zero fill feature is not fully functional yet.
    It ensures the data collection will finish but data probably won't be usable. Increase the rcvBUFmult parameter size if
    experiencing packet loss at mmWave object instantiation. 

    Once rawData is extraced from UDP packet. Each byte is added to the class buffer. It is recommended to run this function
    in its own thread to run concurrently with main program execution. See mmWaveClass_test.py for usage example.
    """
        if not self.captureStarted:
            return # exit function call if capture not started
        try:
            msg, server = self.data_socket.recvfrom(1466) # get data from socket

        except Exception as e:
            return # exit fucntion call if exception rasied

        prevPackNum = self.packNum # store previous packet number
        rawData = msg[10:] # extrat raw data
        self.packNum = int.from_bytes(msg[:4],"little") # extract current packet number
        if (self.packNum - prevPackNum)>1: # check to see if packet number increased by more than 1 = dropped packet
            print(":> Dropped {packetsDropped} packets \n   filling with zeros".format(packetsDropped=(self.packNum - prevPackNum)))
            numZeroBytes = (self.packNum - prevPackNum + 1)*1456 # calculate the number of missing bytes
            for i in range(numZeroBytes): # zero fill buffer with missing bytes
                self.buf.Enqueue(b'\x00')
        for byte in rawData: # add raw data bytes to buffer
            self.buf.Enqueue(byte.to_bytes(1,'little'))

    # %---------------------------------------------------------------------------------------------------------
    def write_Data(self):
        """Function call to extract data from class buffer and write data to file.  

    Function calls getChunk from class buffer to extract a chunk of data for writing to file. 
    
    Every time getChunk is called, the number of bytes written increases. When bytesWritten is equal to frameSize
    the frame counter is increased by 1. When frame counter is equal to the number of expected frames that means all
    frames have been collected and function will stop writing frames and set allFramesCollected flag to true.  

    It is recommended to run this function in its own thread concurrently with collect data. 
    See mmWaveClass_test.py for usage example.

    TO-DO:
    Need to add functionality for set file size write mode and also continuous frame operation. 
    """
        if not self.captureStarted:
            return # exit function call if not 
        if (self.frameCount >= self.configDict["frameCfg"]["numFrames"]) and not self.allFramesCollected:
                    self.allFramesCollected = True
                    print("All frames Collected")
                    print("")
                    print("%-------------------- mmWave Device Raw ADC Data Collection Ended -----------------------%")
                    print("")
                    
        fileName = self.fileName+str(self.fileCounter)+".bin" # set file name
        if not self.allFramesCollected: # ensures more files aren't written if all frames collected
            chunk = self.buf.getChunk() # get dataChunk from buffer
            if not chunk==None: # if chunk was ready
                with open(self.filePath+fileName, "ab") as f: # open file in append binary mode
                    f.write(b"".join(chunk))
                    self.bytesWritten += self.buf.chunkSize
                    if self.bytesWritten >= self.frameSize:
                        timeStamp = time.time()
                        print(":> Frame {frame} Loaded to file @{timeStamp}s".format(timeStamp=timeStamp, frame=self.frameCount))
                        self.fileCounter += 1
                        self.bytesWritten = 0
                        self.frameCount += 1

    # Helper Functions
    # %---------------------------------------------------------------------------------------------------------
    def response_DCA1000(self, cmd):
        """ Helper function to listen for given command response from DCA1000. 

        Will wait for command response. Can cause program to hang if acknowledgement packet is missed. 
        Sets the DCA1000 error flag to true if status is 1. 

        :param string cmd: Name of the command sent to DCA1000 and waiting for response. 
        :returns: status 
        :rtype: int (0 or 1) 0 for success, 1 for fail
        """
        print(">: Waiting for " + cmd + " response...")
        received = False
        while not received:
            try:
                msg, server = self.dca_socket.recvfrom(2048)
                (status,) = struct.unpack('<H', msg[4:6])
                received = True
            except Exception as e:
                print(">: Error-> Exception: " + str(e))
                continue
        if status == 0:
            print(">: " + cmd + " was successful!")
            self.DCA1000_error_flag = False
        else:
            print(">: " + cmd + " was unsuccessful!")
            print(">: Status received: " + str(status))
            self.DCA1000_error_flag = True
        return status

    # %---------------------------------------------------------------------------------------------------------
    def write_Serial(self, cmdString):
        """Helper function that handles serialWrites to radar device.

    Takes in a cmdString and writes each character over UART port with utf-8 encoding.

    :param string cmdString: string that needs to be written over UART.
    """
        for char in cmdString:
            self.iwr_serial.write(char.encode('utf-8'))
  
    # %---------------------------------------------------------------------------------------------------------
    # END OF CLASS
    # %---------------------------------------------------------------------------------------------------------