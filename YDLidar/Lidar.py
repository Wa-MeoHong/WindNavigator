"""
    Date : 2024 / 04 / 11
    Writer : Meohong Shin
    Objective : Activate YDLidarX3
        - Created by modifying 'PyLidar3'
        - 
"""
from serial import Serial
from time import sleep
from math import atan, pi, floor
from enum import Enum

name = "YDLidarX3"

class FrequencyStep(Enum):
    oneTenthHertz=1
    oneHertz=2
    
class YDLidarX3pro:
    """ Deal with YDLidar X3 Pro from https://www.ydlidar.com/ """
    def __init__(self, port, chunk_size=6000, no_value=0):
        # Initialize the Connection and Port, Baudrate
        self._port = port
        self._baudrate = 115200
        self._is_scannig = False
        self._is_connected = False
        self.chunk_size = chunk_size
        self._no_value = no_value
        
    def Connect(self):
        # Start Serial Connection By opening serial Port
        # Return success status True/False
        # Lidar Sensor Connection
        
        try:
            if (not self._is_connected):
                print("Ready to connect Lidar Sensor")
                self._s=Serial(port=self._port, baudrate=self._baudrate)        # Initialize Serial Object (YDLidar X3 pro)
                self._is_connected = True           # Connected Varialbe turn 'True'
                sleep(3)                            # Sleep by Synchronizing
                self._s.reset_input_buffer()        # Initialize Serial Port
                print("Connected!")
                
            # if status is Normal, return True
            if ("YDLidarX3" in str(type(self))):
                self._Stop_motor()
            
            return True
            # print(self.GetDeviceInfo())
            
            # if(self.GetHealthStatus()):
            #     return True
            # else:
            #     # Error Message, Try Reconnect
            #     raise Exception("Device status error. Try Reconnecting Device")
        # if Error, print Error Message
        except Exception as e:
            print(e)
            return False
    
    """ Motor Control """    
    def _Start_motor(self):
        self._s.setDTR(1)
        sleep(0.5)
    def _Stop_motor(self):
        self._s.setDTR(0)
        sleep(0.5)
        
    """ Angle Correction (각도 보정) """
    @classmethod
    def _AngleCorr(cls, dist):
        if dist==0:
            return 0
        else:
            return (atan(21.8*((115.3-dist)/(155.3*dist)))*(180/pi))
    
    """ Endian Hex """
    @classmethod
    def _HexArrToDec(cls, data):
        littleEndianVal = 0
        for i in range(0, len(data)):
            littleEndianVal = littleEndianVal+(data[i]*(256**i))
        return littleEndianVal
    
    """ Calculate Distance """    
    @classmethod
    def _Calculate(cls, d):
        ddict = []
        LSN = d[1]
        Angle_fsa = ((YDLidarX3pro._HexArrToDec((d[2],d[3]))>>1)/64.0)#+YdLidarX4._AngleCorr(YdLidarX4._HexArrToDec((d[8],d[9]))/4)
        Angle_lsa = ((YDLidarX3pro._HexArrToDec((d[4],d[5]))>>1)/64.0)#+YdLidarX4._AngleCorr(YdLidarX4._HexArrToDec((d[LSN+6],d[LSN+7]))/4)
        
        if Angle_fsa < Angle_lsa:
            Angle_diff = Angle_lsa - Angle_fsa
        else:  
            Angle_diff = 360 + Angle_lsa - Angle_fsa
        for i in range(0, 2*LSN, 2):
            # Distance Calculation
            dist_i = YDLidarX3pro._HexArrToDec((d[8+i], d[8+i+1]))/4
            # Ignore Distance zero value, it is Massive Noise When Computing mean of distances for each angle
            if dist_i == 0:
                continue
            
            # Intermadiate angle Solution
            Angle_i_tmp = ((Angle_diff/float(LSN-1))*(i/2))+Angle_fsa
            # Angle Correction
            Angle_i_tmp += YDLidarX3pro._AngleCorr(dist_i)
            if Angle_i_tmp > 360:
                Angle_i = Angle_i_tmp - 360
            elif Angle_i_tmp < 0:
                Angle_i = Angle_i_tmp + 360
            else:
                Angle_i = Angle_i_tmp
            
            # Add Dictionary    
            ddict.append((dist_i, Angle_i))
        return ddict

    """ Calculating CheckSum """
    @classmethod
    def _CheckSum(cls, data):
        try:
            ocs = YDLidarX3pro._HexArrToDec((data[6], data[7]))
            LSN = data[1]
            cs = 0x55AA^YDLidarX3pro._HexArrToDec((data[0], data[1]))^YDLidarX3pro._HexArrToDec((data[2], data[3]))^YDLidarX3pro._HexArrToDec((data[4], data[5]))
            for i in range(0, 2*LSN, 2):
                cs = cs^YDLidarX3pro._HexArrToDec((data[8+i], data[8+i+1]))
            if (cs == ocs):
                return True
            else:
                return False
        except Exception as e:
            return False
    
    """ Data Average """
    @classmethod
    def _Mean(cls, data):
        if (len(data) > 0):
            return int(sum(data)/len(data))
        return 0
    
    def StartScanning(self):
        """ Begin the lidar and returns a generator which returns a dictionary consisting angle(degrees) and distance(meters).
        Return Format : {angle(1):distance, angle(2):distance,....................,angle(360):distance}. """
        if (self._is_connected):
            if (not self._is_scannig):
                self._is_scannig = True
                self._s.reset_input_buffer()
                if ("YDLidarX3" in str(type(self))):
                    self._Stop_motor()
                self._s.write(b"\xA5\x60")
                
                # Synchronize
                sleep(0.5)
                
                self._s.read(7)
                distdict = {}
                countdict = {}
                lastChunk = None
                while self._is_scannig == True:
                    for i in range(0, 360):
                        distdict.update({i:[]})
                    data = self._s.read(self.chunk_size).split(b"\xaa\x55")
                    if lastChunk is not None:
                        data[0] = lastChunk + data[0]
                    lastChunk = data.pop()
                    for e in data:
                        try:
                            if(e[0]==0):
                                if(YDLidarX3pro._CheckSum(e)):
                                    d = YDLidarX3pro._Calculate(e)
                                    for ele in d :
                                        angle = floor(ele[1])
                                        if (angle>=0 and angle<360):
                                            distdict[angle].append(ele[0])
                        except Exception as e:
                            pass
                    for i in distdict.keys():
                        if len(distdict[i]) > 0:
                            distdict[i]=self._Mean(distdict[i])
                        else:
                            distdict[i]=self._no_value
                    yield distdict
            else:
                raise Exception("Device is currently in scanning mode.")
        else:
            raise Exception("Device is not Connected")
        
    def StopScanning(self):
        """ Stop Scanning but serial Connection Alive """
        if (self._is_connected):
            if(self._is_scannig):
                self._is_scannig = False
                self._s.write(b"\xA5\x65")
                sleep(1)
                self._s.reset_input_buffer()
                if ("YDLidarX3" in str(type(self))):
                    self._Stop_motor()
            else:
                raise Exception("Device is not set to scanning mode")
        else:
            raise Exception("Device is not connected")
    
    """ Lidar Health status Check """
    def GetHealthStatus(self):
        """ Return Health status of Lidar
            True : Good
            False : Not Good
        """
        if(self._is_connected):
            if self._is_scannig == True:
                self.StopScanning()
            self._s.reset_input_buffer()
            sleep(0.5)
            self._s.write(b"\xA5\x91")
            sleep(0.5)
            data = self._s.read(10)
            print(data[9])
            if data[9]==0 and data[8]==0 and (data[7]==0 or data[7]==1):
                return True
            else:
                return False
        else:
            raise Exception("Device is not connected")
    
    """ Get Lidar Device """
    def GetDeviceInfo(self):
        """ Return Device Information of Lidar in Dictionary
        {"model_number":model_number,
        "firmware_version":firmware_version,
        "hardware_version":hardware_version,
        "serial_number":serial_number}
        """
        if (self._is_connected):
            if self._is_scannig==True:
                self.StopScanning()
            self._s.reset_input_buffer()
            sleep(0.5)
            self._s.write(b"\xA5\x90")
            sleep(0.5)
            data = self._s.read(27)
            model_number = str(data[7])
            firmware_ver = str(data[9])+"."+str(data[8])
            hardware_ver = str(data[10])
            serial_number = ""
            
            for i in range(11,20):
                serial_number = serial_number+str(data[i])
            return {"model_number":model_number,
                    "firmware_version":firmware_ver,
                    "hardware_version":hardware_ver,
                    "serial_number":serial_number }
        else:
            raise Exception("Device is not connected")
        
    """ Reboot Lidar Sensor """
    def Reset(self):
        if (self._is_connected):
            self._s.write(b"\xA5\x40")
            sleep(0.5)
            self.Disconnect()   # Disconnect
            self.Connect()      # Connect
        else:
            raise("Device is not connected")
        
    """ Disconnect and close Serial Communication """
    def Disconnect(self):
        if(self._is_connected):
            if(self._is_scannig == True):
                self.StopScanning()
            self._s.close()
            self._is_connected = False
        else:
            raise Exception(" Device is not Connected ")