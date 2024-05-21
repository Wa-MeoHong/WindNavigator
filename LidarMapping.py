"""
    File : LidarTest.py
    Writer : MeoHong
    Date : 2024 / 05 / 13
    Objective : Lidar Scanning, and Carculate Room width, length, Mapping Window location
    Modify : 
    ==============================================================================
        1   |   05/21   | 최초 작성
        2   |   05/22   | 30초 스캔 후, 프레임 정지
        3   |   05/22   | 창문 매핑을 위해 모드 체인지 기능 추가 (s키로 동작)
        4   |   05/22   | 창문의 위치를 각 모드에 맞게 상대적 수직 거리를 계산해서, 딕셔너리에 담는 것을 구현
        5   |   05/22   | 창문의 위치를 
    ============================================================================== 
"""

from YDLidar import *
import os
import sys
import cv2
import numpy as np
import time
import math

RECORD_POINTS = []
WINDOW_LOCA = []

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
        cv2.line(frame, CENTER, (x, y), (255,255,255), 1)
        
def calculate_room_dimensions(dict_scan):
    angles = dict_scan['angle']
    distances = dict_scan['dist']
    
    # 점들을 전부 직교좌표계의 점으로 표현
    points = np.array([polar_to_cartesian(angle, dist) for angle, dist in zip(angles, distances)])
    
    min_x = np.min(points[:, 0]) # 행렬의 1번째 열의 값이 X좌표
    max_x = np.max(points[:, 0])
    min_y = np.min(points[:, 1]) # 행렬의 2번째 열의 갑이 Y좌표
    max_y = np.max(points[:, 1])
    # print(points)
    
    # 가로, 세로 길이 계산
    width = max_x - min_x
    height = max_y - min_y
    
    return (width, height)

# 모드 체인지
# 순환해야 하기 때문에, 나머지를 이용한 변형을 사용했다.
def mode_change(current_mode):
    modes = ["win_left", "win_top", "win_right", "win_bottom"]
    current_index = modes.index(current_mode)
    next_index = (current_index + 1) % len(modes)
    return modes[next_index]

# OpenCV 마우스 이벤트 조작 
# 좌클릭 : 좌표 입력, 단, 기록된 좌표는 2개가 한계. 그 이상 그이하 절대 안됨
# 우클릭 : 잘못 기록하였을 경우가 있어, 기록을 초기화함.
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(RECORD_POINTS) < 2:
            print(f"Left Clicked coordinates : ({x}, {y})")
            RECORD_POINTS.append((x, y))
        else:
            print(f"Recorded points is full")
        print(f"Recorded points : {RECORD_POINTS}")
        
    elif event == cv2.EVENT_RBUTTONDOWN:
        if len(RECORD_POINTS) == 0:
            print("NO Recorded points")
        else:
            RECORD_POINTS.clear()
            print("Clear Recorded points")

def Window_locations(windows_dict, current_mode):
    if len(RECORD_POINTS) != 2:
        print("Error! Recorded Point not enough")
        return
    
    recorded_points = RECORD_POINTS
    
    # 현재 점의 위치는 OpenCV 좌표에 표시하기 위해, 스케일링 되어있기 때문에
    # 미터 호환을 위해 다시 리스케일링한다.
    rescaled_point = ([(((x - CENTER[0]) / SCALE), ((y - CENTER[1]) / SCALE)) for x, y in recorded_points])
    #print(rescaled_point)
    
    # 두 점 사이의 거리를 구한다. 이는 미터로 환산된다.
    # 유클리드 거리로 판단한다. 
    x1, y1 = rescaled_point[0]
    x2, y2 = rescaled_point[1]
    distance = round(math.sqrt((x2-x1)**2 + (y2-y1)**2), 3)
    print(distance)
    
    # 각 모드에 맞게, 그 키의 리스트에 추가한다.
    windows_dict[current_mode].append(distance)
    print(windows_dict)
    
    # 거리 계산이 끝났으면, 기록된 위치를 초기화한다.
    # 계산이 끝난 좌표는 기록을 위해, WINDOW_LOCA에 저장한다.
    WINDOW_LOCA.append((RECORD_POINTS[1], current_mode))
    RECORD_POINTS.clear()
    
    return 
# 라이다 센서 작동 시작
port = 'COM5'
Lidar = YDLidarX3Pro(port=port,ScanFreq=10.0)
scan = Lidar.setting()
ret = Lidar.Turn_On()
current_mode = "win_left"



# 창문 딕셔너리 초기화
windows_dict = {
    "win_left": [],
    "win_top": [],
    "win_right": [],
    "win_bottom": []
    }
# print(windows_dict)

if ret:
    start_time = time.time()
    last_frame = None
    
    # 30초 스캔
    while time.time() - start_time < 10 :
        dict_scan = Lidar.Scan_point(scan=scan)
        frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        draw_lidar(frame, dict_scan['angle'], dict_scan['dist'], dict_scan['intensity'])
        cv2.imshow("Lidar", frame)
        last_frame = frame
        
        # 미리 정지 할 수 있음
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    Lidar.Turn_Off()
Lidar.Disconnect()

# 방의 가로, 세로 길이를 구하는 작업
#print(dict_scan)
room_data = calculate_room_dimensions(dict_scan)
print(room_data)

# 마지막 프레임을 유지하여, OpenCV창을 닫지 않게 한다
# 이는 다음 창문 좌표를 매핑하는 것을 위한 초석이 된다. 
print(f"Current_mode : {current_mode}")
cv2.namedWindow("Lidar")
cv2.setMouseCallback("Lidar", click_event)

while True: 
    if last_frame is not None:
        frame_with_clicks = last_frame.copy()
        
        for point in WINDOW_LOCA:
            if point[1] == "win_left":
                cv2.circle(frame_with_clicks, point[0], 8, (0, 0, 255), -1)     # 빨간색 원형 좌표점
            elif point[1] == "win_top":
                cv2.circle(frame_with_clicks, point[0], 8, (0, 255, 255), -1)     # 노란색 원형 좌표점
            elif point[1] == "win_right":
                # 연보라색 사각형 점
                top_left = (point[0][0] - 7, point[0][1] - 7)
                bottom_right = (point[0][0] + 7, point[0][1] + 7)
                cv2.rectangle(frame_with_clicks, top_left, bottom_right, (255, 154, 208), -1)
            elif point[1] == "win_bottom":
                # 시안색 사각형 점
                top_left = (point[0][0] - 7, point[0][1] - 7)
                bottom_right = (point[0][0] + 7, point[0][1] + 7)
                cv2.rectangle(frame_with_clicks, top_left, bottom_right, (210, 163, 0), -1)
        cv2.imshow("Lidar", frame_with_clicks)
        
    key = cv2.waitKey(1) & 0xFF
    # 나가기 (Quit) 
    if key == ord('q'):
        break
    
    # 모드 스위치 (Switch)
    elif key == ord('s'):
        current_mode = mode_change(current_mode)
        print(f"Current_mode : {current_mode}")
    
    # 계산하기 (Carculate)
    elif key == ord('c'):
        Window_locations(windows_dict, current_mode)
    
cv2.destroyAllWindows()