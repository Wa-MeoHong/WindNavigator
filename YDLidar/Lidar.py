"""
    File Name : Lidar.py ( for YDLidarX3Pro )
    Writer : Meo Hong
    Date : 2024 / 05 / 16 ( Wed.)
    Objective : For Lidar, YDLidar X3 Pro initialize
    Modify Detail:
    ========================================================================
        1   |   05/15   |   First Write
        2   |   05/16   |   Write Class, test code
        3   |   ...     |   ...
    ========================================================================
    
"""

import ydlidar

class ParameterFalseException(Exception):
    pass

""" YDLidar SDK를 이용한 클래스를 새로 작성 """
class YDLidarX3Pro:
    def __init__(self, port, baudrate=115200, ScanFreq=10.0, SampleRate=4, singlechannel=True) -> None:
        # 포트, Baudrate, 샘플링 주파수 (1000단위), 스캔 주파수(1000단위, Hz)를 설정
        self._port = port
        self._baudrate = baudrate
        self._samplerate = SampleRate
        self._scanfreq = ScanFreq
        self._singlechannel = singlechannel
        self.lidar = ydlidar.CYdLidar()

    def setting(self):
        # 라이다에 __init__으로 정의한 변수들을 집어넣고, 스캔할 변수를 반환
        self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, self._port)
        self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, self._baudrate)
        self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, self._scanfreq)
        self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, self._samplerate)
        self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, self._singlechannel)
        scan = ydlidar.LaserScan()
        return scan
        
    def Turn_On(self):
        # lidar initialize 한 후, 전원을 키는 동작
        # 만약 초기화가 되지 않거나, 전원이 이미 켜져있다면 False를 반환
        # 전원을 키는 동작이 제대로 이루어진다면 True
        ret = self.lidar.initialize()
        if not ret:
            return False
        ret = self.lidar.turnOn()
        
        if not ret:
            return False
        return True
    
    def Turn_Off(self):
        # 전원을 끄는 동작. 전원을 끄고 연결을 끊는다.
        self.lidar.turnOff()
    def Disconnect(self):
        self.lidar.disconnecting()
        
    def Scan_point(self, scan):
        r = self.lidar.doProcessSimple(scan)
        if r:
            angles = []
            distances = []
            intensity = []
            for point in scan.points:
                angles.append(point.angle)
                distances.append(point.range)
                intensity.append(point.intensity)
            
            dict_scan = {
                'angle' : angles,
                'dist' : distances,
                'intensity' : intensity
            }
            return dict_scan
        else:
            return None