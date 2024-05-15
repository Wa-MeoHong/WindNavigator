from YDLidar import *
import os
import sys
import cv2
import numpy as np

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
        y = int(CENTER[1] + y * SCALE)
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