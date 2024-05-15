from serial import Serial
from time import sleep
import numpy as np
import math
import sys
import math
import matplotlib.pyplot as plt
import cv2
import threading

port = "/dev/ttyUSB0"
name = "YDLidarX3"
    
class YDLidarX3pro:
    def __init__(self, port, bardrate=115200):
        self._port = port
        self._bardrate = bardrate
        self._is_connected = False
        self._packetbyte = 90
        
    # 연결
    def Connect(self):
        try:
            if(not self._is_connected):
                print("Get Ready to Connect ")
                # 시리얼 통신 포트 열기
                self.ydlidarX3Pro = Serial(
                    port=self._port,
                    baudrate=self._bardrate,
                )
                self._is_connected = True
                sleep(3)
                print("Connected!")
                self.ydlidarX3Pro.reset_input_buffer() # 버퍼 리셋
                print("Initialized, Reset input Buffer.")
                
            self.scan_code = bytes([0xA5, 0x5A, 0x05, 0x00, 0x40, 0x81])
            self.ydlidarX3Pro.write(self.scan_code)
            
            self.ydlidarX3Pro.close()       # 나중에 다시 열 것임
            
        except Exception as e:
            print(e)
            self.ydlidarX3Pro.close()
            sys.exit()
            
    def Open(self):
        self.ydlidarX3Pro.open()
    def Close(self):
        self.ydlidarX3Pro.close()
    
    def DisConnect(self):
        if self._is_connected:
            self.ydlidarX3Pro.close()
            self._is_connected = False
        else:
            raise Exception(" Device is not Connected ")
            
    def _GetData(self):
        PH_LSB = b'\xAA'
        PH_MSB = b'\x55'
        CT = '\x00'
        LS = '\x28'
        
        is_right_packet = False

        # 패킷의 헤더를 읽고, 다음을 만족하지 못하면 버린다
        # 패킷헤더만 확인한다는 개념
        while True:
            if self.ydlidarX3Pro.read() == PH_LSB:
                if self.ydlidarX3Pro.read() == PH_MSB:
                    if self.ydlidarX3Pro.read() == CT:
                        if self.ydlidarX3Pro.read() == LS:
                            is_right_packet = True
                            break
        
        data = []
        if is_right_packet == True:
            # 헤더 부분을 제외한 나머지 부분 86Byte를 읽어옴
            raw_data = self.ydlidarX3Pro.read(self._packetbyte - 4)
            
            data.append(PH_LSB + (PH_MSB << 8))
            data.append(CT + (LS << 8))
            
            i = 0
            while i in len(raw_data):
                data.append(raw_data[i] + (raw_data[i+1] << 8))
            
        return data
        
        """while True:
            self.ydlidarX3Pro.open()
            raw_data = bytearray(self.ydlidarX3Pro.read(self._packetbyte))
            
            # 참고로 Little Endian으로 보내지고 있어서, Big Endian으로 변환해야함
            # Packet Header와 Package Type(CT), Sample quantity (LSN)을 살펴보기 위해 다음을 작업
            # 바이트 단위로 쪼개지므로, 인덱싱이 가능하며, 비트 연산을 통해 합해서 패킷의 정보를 알아낼 수 있음 
            ph_temp = raw_data[0]
            ph_temp += raw_data[1] << 8
            ctlsn_temp = raw_data[2]
            ctlsn_temp += raw_data[3] << 8
            
            # 만약 패킷의 시작이 0x55AA가 아니라면 바로 제외시킴 (이는 X4에서나, X2에서도 동일하게 나왔음)
            # Ctlsn은 CT의 비트 0번이 0일 경우, 라이다 사이클의 시작을 의미함
            # lsn은 샘플링 포인터의 개수를 의미한다.
            
            # 현재 첫 패킷만 데이터를 받아들이고, 나머지는 읽지 않겠다는 것
            if ph_temp == 0x55AA and ctlsn_temp == 0x2800:
                i = 0
                self.data = []
                # 2바이트씩 묶어서 값을 보내고 있으므로, 모든 데이터는 2바이트씩 같이 묶어주고 데이터를 해석해야한다.
                while i < self._packetbyte:
                    temp = raw_data[i]
                    temp += raw_data[i+1] << 8
                    self.data.append(temp)
                    i += 2
                if(YDLidarX3pro._Checksum(self.data)):
                    break
                else:
                    continue
            else:
                continue"""
            
    @classmethod
    def _Checksum(cls, data):
        try: 
            # 체크코드 계산, 2Byte씩 묶여있고, XOR연산을 통해 CheckSum을 만들었다고 함.
            ocs = data[4]
            cs = 0
            for i in range(4):
                cs ^= data[i]
            for i in range(5,len(data)):
                cs ^= data[i]
            
            if (cs == ocs):
                return True
            return False 
            
        except Exception as e:
            return False 
    
    # 각도 보정 함수
    @classmethod
    def _AngleCorr(cls, dist):
        if dist == 0:
            return 0
        return math.degrees(math.atan(21.8*(155.3-dist)/(155.3*dist)))
    
    # 계산 함수
    @classmethod
    def _Calculate(cls, data):
        an_fsa = (data[2] >> 1) // 64   # 라이다 패킷의 현재 패킷의 시작 위치 각도
        an_lsa = (data[3] >> 1) // 64   # 현재 패킷의 끝 위치 각도
        lsn = data[1] >> 8      # 샘플링 포인트의 개수
        
        if an_fsa > an_lsa:
            ang_diff = abs(an_fsa - (360 + an_lsa)) # 각도는 양수로 계산할 것
        else:
            ang_diff = abs(an_fsa - an_lsa)
        
        # 체크섬과 나머지 모든 부분은 전부 가져옴
        sampled_data = data[5:]
        distance = list(map(lambda x : x // 4, sampled_data))
        
        angle = [an_fsa]
        for i in range(1, lsn - 1):
            angle.append((ang_diff/(lsn-1) * i)+ an_fsa)
        angle.append(an_lsa)
        
        ang_correct = []
        for cor in range(lsn):
            if distance[cor] == 0:
                ang_correct = 0
                angle[cor] = 0
            else:
                # 각도 보정을 함
                ang_correct = YDLidarX3pro._AngleCorr(distance[cor])
                angle[cor] = abs(angle[cor]-ang_correct)
            
            # 각도 보정 후, 360도 각도계에 맞춤
            if angle[cor] >= 360:
                angle[cor] = angle[cor] - 360
            else:
                angle[cor] = angle[cor]
        
        return angle, distance

    def Scanning(self):
        self.Open()
        data = self._GetData()
        if not self._Checksum(data):
            return 0
        angle, dist = YDLidarX3pro._Calculate(data)
        return angle, dist
    
def draw():
    global is_plot
    while is_plot:
        plt.figure(1)
        plt.cla()
        plt.ylim(-500, 500)
        plt.xlim(-500, 500)
        plt.scatter(x, y, c='r', s=8)
        plt.pause(0.001)
    plt.close("all")
    
is_plot = True
x=[]
y=[]
for _ in range(360):
    x.append(0)
    y.append(0)

    
        
def main():
    ydlidarX3 = YDLidarX3pro(port=port)
    ydlidarX3.Connect()
    threading.Thread(target=draw).start()
        
    while True:
        angle, distance = ydlidarX3.Scanning()
        for idx in range(len(distance)):
            x[idx] = distance[idx]/15 * math.cos(math.radians(angle[idx]))
            y[idx] = distance[idx]/15 * math.sin(math.radians(angle[idx]))

if __name__ == "__main__":
    
    main()