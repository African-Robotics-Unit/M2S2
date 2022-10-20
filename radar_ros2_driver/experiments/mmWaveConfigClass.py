# Standard Python Libray
import json # library for writing and parsing dicts to json files 

class mmWaveConfig():

    # %---------------------------------------------------------------------------------------------------------
    # Class Parameters
    # %---------------------------------------------------------------------------------------------------------
    # Runtime dict stores parameters for how data and commands are sent to and received from device at runtime
    # equivalent to mmwaveConfig.txt file in TI mmwave platforms tool box
    runtimeParamDict = None

    # mmWave dict stores device parameters for configuration of waveform, adc and data port outputs etc. 
    # equivalent to profile_monitor_xwrXXXX.cfg files in TI mmwave platforms tool box
    mmwaveParamDict = None

    # list of available config commands for mmwaveDevice used to check contents of json are correct
    # key is command, value is number of settings which is equal to len(settingsDict)
    commandList = {'dfeDataOutputMode' : 1, 
                    'channelCfg' : 3, 
                    'adcCfg': 2, 
                    'adcbufCfg':5, 
                    'profileCfg':14, 
                    'chirpCfg_0':8, 
                    'chirpCfg_1':8, 
                    'frameCfg':7, 
                    'lowPower':2, 
                    'lvdsStreamCfg':4, 
                    'calibMonCfg':2, 
                    'monCalibReportCfg':3, 
                    'txPowerMonCfg_0':3, 
                    'txPowerMonCfg_1':3, 
                    'txPowerMonCfg_2':3, 
                    'txBallbreakMonCfg_0':2, 
                    'txBallbreakMonCfg_1':2, 
                    'txBallbreakMonCfg_2':2, 
                    'rxGainPhaseMonCfg':2, 
                    'tempMonCfg':2, 
                    'synthFreqMonCfg':2, 
                    'pllConVoltMonCfg':1, 
                    'dualClkCompMonCfg':1, 
                    'rxIfStageMonCfg':2, 
                    'extAnaSigMonCfg':1, 
                    'pmClkSigMonCfg':2, 
                    'rxIntAnaSigMonCfg':2, 
                    'gpadcSigMonCfg':1}

    # list of available runtime config options used to check runtime file format correctness
    runtimeList = ['VARIANT', 
                    'COMMAND_PORT', 
                    'DATA_PORT', 
                    'CONFIG_FILE_FORMAT', 
                    'CAPTURED_ADC_DATA_PATH', 
                    'DCA_FILE_PREFIX', 
                    'DCA_MAX_REC_FILE_SIZE_MB', 
                    'DCA_DATA_FORMAT_MODE', 
                    'DCA_LVDS_LANE_MODE']

    # %---------------------------------------------------------------------------------------------------------
    # Class Functions
    # %---------------------------------------------------------------------------------------------------------
    # Class Constructor
    def __init__(self, runtimeConfigPath = None, mmwaveConfigPath = None):
        """Initializes the config object which stores all required configuration parameters.
        
        Function either initializes configs to default values or loads files from json. All 
        configs are stored as dictionaries for easy access to values. 
        Dictionaries are accessed by called mmWaveConfigObj.runtimeParamDict for runtime parameters and
        mmWaveConfigObj.mmwaveParamDict for mmwave sensor parameters
            
        :param string runtimeConfigPath: the path to mmWaveRuntimeConfig.json  (Default = None, default configs will be loaded)
        :param string mmwaveConfigPath: the path to mmWaveDeviceConfig_XWRXXXX.json   (Default = None, default configs will be loaded)
        """
        print("%--------------------- Loading mmWave Device and Runtime Configs ------------------------%")
        if runtimeConfigPath == None:
            # default values stored in class (can use as a template for making your own JSONs)
            self.runtimeParamDict =   {
                        'VARIANT' : "IWR6843",                              # radar board type
                        'COMMAND_PORT' :  "/dev/ttyACM1",                           # UART command port designation
                        'DATA_PORT'    :  "/dev/ttyACM2",                           # UART data port designation
                        'CONFIG_FILE_FORMAT':0,                             # Config file format 1-cfg, 0-json (only json supported currently)
                        'CAPTURED_ADC_DATA_PATH':"captured_adc_IWR6843",    # folder path for raw data storage 
                        'DCA_FILE_PREFIX':"adc_data",                       # ADC data bin file name prefix
                        'DCA_MAX_REC_FILE_SIZE_MB':0,                       # ADC data bin file max size in MB, set to zero to create a file for each frame
                        'DCA_DATA_FORMAT_MODE':3,                           # 1: 12bit, 2: 14bit, 3:16bit
                        'DCA_LVDS_LANE_MODE':2,                             # 2: 2 Lane LVDS, 4: 4 Lane LVDS
                        }
            print(":> Done-> runtimeConfig loaded successfully")
        else:
            # if path given, instead set runtimeDict to parsed json file
            self.parse_runtimeConfigJSON(runtimeConfigPath)

        if mmwaveConfigPath == None:
            # default values stored in class (can use as a template for making your own JSONs)
            self.mmwaveParamDict =   {
                        'dfeDataOutputMode':
                            {'frmMode'          : 1},       # modeType 1 = frameBased Chirps, 2 = continuous chirping, 3 = advanced frame config

                        'channelCfg':
                            {'rxChannelEn'      : 15,       # bit mask 15 = 1111 i.e. takes values 1-15
                             'txChannelEn'      : 7,        # bit mask 7 =  111  i.e. takes values 1-7
                             'casMode'          : 0},       # 0-en, 1-dis

                        'adcCfg':
                            {'numADCBits'       : 2,        # 0=12, 1=14, 2=16
                             'adcOutputFmt'     : 1},       # 0=real, 1=complex1, 2=complex2
                        
                        'adcbufCfg':
                            {'subFrameIdx'      : -1,       # -1 for legacy mode
                             'adcOutputFmt'     : 0,        # 0=complex, 1=real
                             'SampleSwap'       : 1,        # 0 = I in LSB, Q in MSB ; 1 = Q in LSB, I in MSB
                             'chanInterleave'   : 1,        # 0 = Interleaved (XWR14xx only), 1 = not-interleaved
                             'chirpThreshold'   : 1},       # 0-8 = DSP for 1D FFT, 1 = HWA for 1D FFT

                        'profileCfg':
                            {'profileId'        : 0, 
                             'startFreq'        : 60,       # in GHz
                             'idleTime'         : 117,      # in us
                             'adcStartTime'     : 7,        # in us
                             'rampEndTime'      : 13.12,    # in us
                             'txOutPower'       : 0,        # in dB
                             'txPhaseShifter'   : 0, 
                             'freqSlopeConst'   : 38.11,    # in MHz/us 
                             'txStartTime'      : 1,        # in us
                             'numAdcSamples'    : 64,       
                             'digOutSampleRate' : 12500,    # ksps 
                             'hpfCornerFreq1'   : 0,        # hpf1 0=175KHz, 1=235KHz, 2=350KHz, 3=700KHz
                             'hpfCornerFreq2'   : 0,        # hpf2 0=350KHz, 1=700KHz, 2=1.4MHz, 3=2.8MHz
                             'rxGain'           : 158},     # in dB

                        'chirpCfg_0':
                            {'chirpStartIndex'  : 0,        # 0-511
                             'chirpEndIndex'    : 0,        # 0-511
                             'profileId'        : 0,        # match profileCfg -> profileID
                             'startFreqVar'     : 0,        
                             'freqSlopeVar'     : 0,
                             'idleTimeVar'      : 0, 
                             'ADCStartTimeVar'  : 0, 
                             'chirpAntennaEnMask': 1},

                        'chirpCfg_1':
                            {'chirpStartIndex'  : 1,        # 0-511
                             'chirpEndIndex'    : 1,        # 0-511
                             'profileId'        : 0,        # match profileCfg -> profileID
                             'startFreqVar'     : 0,        
                             'freqSlopeVar'     : 0,
                             'idleTimeVar'      : 0, 
                             'ADCStartTimeVar'  : 0, 
                             'chirpAntennaEnMask': 4},

                        'frameCfg':
                            {'chirpStartIndex'  : 0,        # 0-511
                             'chirpEndIndex'    : 1,        # 0-511
                             'numChirps'        : 32,       # 1-255 chirps
                             'numFrames'        : 100,      # 0-65535
                             'framePeriod'      : 100,      # in ms
                             'triggerSelect'    : 1,        # 1=Software, 2=hardware
                             'frameTriggerDelay': 0},       # in 
                             
                        'lowPower':
                            {'dontCare'         : 0,        # always 0
                             'adcMode'          : 0},       # 0=regular, 1=lowPower
                        
                        'lvdsStreamCfg':
                            {'subFrameIdx '     : -1,       # -1 for legacy
                             'enableHeader'     : 0,        # 1=HSI Header enabled, 0 HSI Header disabled
                              'dataFmt'         : 1,        # 0=HW Streaming Disabled, 1=ADC, 4=CP_ADC_CQ
                              'enableSW'        : 0},       # 0=Disable User Data, 1=Enable User Data (HSI header must be enabled)
                        
                        # entries after here are fairly arbitary leaving them as default as given by TI mmwave platforms tool box

                        'calibMonCfg':
                            {'calibMonTimeUnit' : 1,        # 0  periodic Calibration and monitoring is DISABLED, else valid value (>0) to enable.
                             'calibPeriodicity' : 1},       # Calibration periodicity calibPeriodicity = 0: to disable periodic calibration, value (>0) to set the calibration period.

                        'monCalibReportCfg':
                            {'enableCalibReport': 1,        # Enable the periodic calibration reports 
                             'enableFailureReport': 1,      # Enable Failure report, 0=disabled, 1=BSS->UART, n>1 sent every Nth Frame
                             'reserved'         : 0},       # reserved? just keep 0

                        'txPowerMonCfg_0':
                            {'enable'           : 1,        # Enable Tx Power monitor
                             'TxAnt'            : 0,        # Which antenna is enabled 0-2 for three antennas
                             'profileIdx'       : 0},       # Profile ID which will be used for this monitor

                        'txPowerMonCfg_1':
                            {'enable'           : 1,        # Enable Tx Power monitor
                             'TxAnt'            : 1,        # Which antenna is enabled 0-2 for three antennas
                             'profileIdx'       : 0},       # Profile ID which will be used for this monitor

                        'txPowerMonCfg_2':
                            {'enable'           : 1,        # Enable Tx Power monitor
                             'TxAnt'            : 2,        # Which antenna is enabled 0-2 for three antennas
                             'profileIdx'       : 0},       # Profile ID which will be used for this monitor

                        'txBallbreakMonCfg_0':              # ball break monitor controller 
                            {'enable'           : 1,        # Enable Tx Ballbreak monitor
                             'TxAnt'            : 0},       # Which antenna is enabled 0-2 for three antennas

                        'txBallbreakMonCfg_1':              # ball break monitor controller 
                            {'enable'           : 1,        # Enable Tx Ballbreak monitor
                             'TxAnt'            : 1},       # Which antenna is enabled 0-2 for three antennas

                        'txBallbreakMonCfg_2':              # ball break monitor controller 
                            {'enable'           : 1,        # Enable Tx Ballbreak monitor
                             'TxAnt'            : 2},       # Which antenna is enabled 0-2 for three antennas

                        'rxGainPhaseMonCfg':                # rx gain phase monitor
                            {'enable'           : 1,        # enable monitor
                             'profileIdx'       : 0},       # monitor ID

                        'tempMonCfg':                       # Temperature monitor
                            {'enable'           : 1,        # enable monitor
                             'tempDiffThresh'   : 20},      # Temperature difference threshold  

                        'synthFreqMonCfg':                  # synth frequency monitor
                            {'enable'           : 1,        # Enable Tx Ballbreak monitor
                             'profileIdx'       : 0},       # monitor ID

                        'pllConVoltMonCfg':                 # apll + synth control voltage monitor
                            {'enable'           : 1},       # Enable monitor

                       'dualClkCompMonCfg':                 # Dual clock compare monitor
                            {'enable'           : 1},       # Enable monitor

                        'rxIfStageMonCfg':                  # RX filter attenuation monitor
                            {'enable'           : 1,        # enable monitor
                             'profileIdx'       : 0},       # monitor ID
                        
                       'extAnaSigMonCfg':                   # This command is to control External DC signals Monitor 
                            {'enable'           : 0},       # Enable monitor

                        'pmClkSigMonCfg':                   # Control Power management, Clock, LO distribution circuits internal analog sig monitor
                            {'enable'           : 1,        # enable monitor
                             'profileIdx'       : 0},       # monitor ID
                        
                        'rxIntAnaSigMonCfg':                # control RX internal signals monitor
                            {'enable'           : 1,        # enable monitor
                             'profileIdx'       : 0},       # monitor ID

                       'gpadcSigMonCfg':                    # This command is to control GPADC Internal Analog signals monitor
                            {'enable'           : 1},       # Enable monitor
                        }
            print(":> Done-> mmWaveConfig loaded successfully")
        else:
            # if path given, instead set mmwaveDict to parsed json file
            self.parse_mmWaveConfigJSON(mmwaveConfigPath)
        print("")

    # Primary Functions
    # %---------------------------------------------------------------------------------------------------------
    def write_sensorConfigJson(self):
        """Writes a new sensor json file based on parameters currently stored in configs object.
        
        Look at json file and default dictionary structure and mmWaveConfigClass_test.py for examples on how to change params. 
        Otherwise just change default params in class and call init method wih corresponding file path set 
        to none and then call this function to write values to json file. Need to make sure runtimeConfig has been loaded
        first so board variant can be accessed by classed.   
        """
        with open("mmWaveDeviceConfig_{variant}.json".format(variant=self.runtimeParamDict['VARIANT']), "w") as outfile:
            json.dump(self.mmwaveParamDict, outfile, indent=3)

    # %---------------------------------------------------------------------------------------------------------
    def write_runtimeConfigJson(self):
        """Writes a new runtime json file based on parameters currently stored in configs object.
        
        Look at json file and default dictionary structure and mmWaveConfigClass_test.py for examples on how to change params. 
        Otherwise just change default params in class and call init method wih corresponding file path set 
        to none and then call this function to write values to json file.
        """
        with open("mmWaveRuntimeConfig.json", "w") as outfile:
            json.dump(self.runtimeParamDict, outfile, indent=1)

    # %---------------------------------------------------------------------------------------------------------
    def parse_mmWaveConfigJSON(self,filePath):
        """Loads an already written and correctly formatted json file into config object.
        
        File path given during initialization is used to load file. Function checks if file is correctly formatted 
        and states where errors in formatting occured. Function is very rigid on Json formatting and dictionary stored in
        json needs to follow default dictionary format given in class constructor exactly. 

        :params string filePath: filePath to json file to loaded into class.
        """
        with open(filePath) as json_file:
            loadedDict = json.load(json_file) # load config file from json format
            # check if keys are correct
            error = False
            if list(loadedDict.keys())==list(self.commandList.keys()): # check if correct commands present in file
                for dictKey in loadedDict:
                    if len(loadedDict[dictKey])!=self.commandList[dictKey]:
                        error=True
                        print(":> Error-> {command} has incorrect number of settings".format(command=dictKey))
                        print("\t{command} requires {numCmd} settings".format(command=dictKey,numCmd=self.commandList[dictKey]))
                        print("")

            else:
                error=True
                print(":> ERROR-> missing following commands from JSON file:")
                diff = list(set(list(self.commandList.keys()))-set(list(loadedDict.keys())))
                for missing in diff:
                    print("\t"+missing)
                print("")

            if error:
                self.mmwaveParamDict = None
                print(":> Error-> {file} is formatted incorrectly".format(file=filePath))
                print("\tSetting mmWaveConfigDict to None")
                print("")
            else:
                print(":> Done-> mmWaveConfig loaded successfully")
                self.mmwaveParamDict = loadedDict

    # %---------------------------------------------------------------------------------------------------------
    def parse_runtimeConfigJSON(self,filePath):
        """Loads an already written and correctly formatted json file into config object.
        
        File path given during initialization is used to load file. Function checks if file is correctly formatted 
        and states where errors in formatting occured. Function is very rigid on Json formatting and dictionary stored in
        json needs to follow default dictionary format given in class constructor exactly. 

        :params string filePath: filePath to json file to loaded into class.
        """
        with open(filePath) as json_file:
            loadedDict = json.load(json_file)
            if self.runtimeList == list(loadedDict.keys()):
                print(":> Done-> runtimeConfig loaded successfully")
                self.runtimeParamDict = loadedDict
            else:
                print(":> ERROR-> missing following commands from JSON file:")
                diff = list(set(self.runtimeList)-set(list(loadedDict.keys())))
                for missing in diff:
                    print("\t"+missing)
                print("")
                self.runtimeParamDict = None
                print(":> Error-> {file} is formatted incorrectly".format(file=filePath))
                print("\tSetting runtimeConfigDict to None")
                print("")

    # %---------------------------------------------------------------------------------------------------------
    def get_mmWaveCommandString(self, cmd):
        """Takes in a command such as "channelCfg" which corresponds to a key in mmwaveParamDict and 
        returns the complete command as string will all setting such as "channelCfg 15 7 0\\n".
        
        Creates a list out the available keys in mmwaveParamDict and checks if cmd is in that list.
        If it is the command is valid and it extracts the settings dictionary associated with that command from
        mmwWaveParamDict. Function then removes any suffixes not supproted by the mmWaveDevice such as _0, _1 etc. 
        that need to be used in the json file to separate different keys that represent the same command. 
        Then adds each setting to command string to be returned and adds the \\n terminator to end of string. 

        :params string cmd: config command name
        :returns string cmdStr: full command with command name and all settings and terminator
        :rtype: string  
        """
        if cmd in list(self.mmwaveParamDict.keys()):
            settingsDict = self.mmwaveParamDict[cmd]
            cmdStr = cmd.split("_", 1)[0] # remove repeated command suffix
            for setting in settingsDict:
                cmdStr = cmdStr + " " + str(settingsDict[setting]) # append each setting with a space to command string
            cmdStr = cmdStr + "\n"
            return cmdStr
        else:
            print(":> Invalid Command")
            return ""
    

