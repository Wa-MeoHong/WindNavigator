"""
    File Name : Test4.py ( for lidar test )
    Writer : Meo Hong
    Date : 2024 / 05 / 16 ( Wed.)
    Objective : For Lidar test, YDLidar X3 Pro initialize
    Modify Detail:
    ========================================================================
        1   |   05/15   |   First Write
        2   |   05/16   |   Write Class, test code
        3   |   ...     |   ...
    ========================================================================
    
"""

"""
    이번 클래스는 저번과 다르게, YDLidar-SDK를 사용해서 다시한번 더 작성한다.
    다른 점은 파이썬 내에서 직접 패킷해석을 거치지 않는다. 그리고 잘나옴
"""
import ydlidar
import numpy as np
import os
import time
import cv2

RMAX = 8.0
WIDTH = 1200
HEIGHT = 1200
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = WIDTH / (2 * RMAX)

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
            
# 좌표를 극좌표로 계산하여 다시 반환
def polar_to_cartesian(angle, dist):
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return x, y

# OpenCV를 이용해 좌표를 계산함.
def draw_lidar(frame, angles, dists, intensities):
    for angle, dist, intensity in zip(angles, dists, intensities):
        x, y = polar_to_cartesian(angle, dist)
        x = int(CENTER[0] + x * SCALE)
        y = int(CENTER[1] - y * SCALE)
        color = (0, int(255 * intensity/255.0), int(255 * (1 - intensity / 255.0)))
        cv2.circle(frame, (x, y), 2, color, -1)

port = 'COM5'
Lidar = YDLidarX3Pro(port=port)
scan = Lidar.setting()
ret = Lidar.Turn_On()
if ret:
    while True:
        dict_scan = Lidar.Scan_point(scan=scan)
        frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        draw_lidar(frame, dict_scan['angle'], dict_scan['dist'], dict_scan['intensity'])
        cv2.imshow("Lidar", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    Lidar.Turn_Off()
Lidar.Disconnect()
cv2.destroyAllWindows()