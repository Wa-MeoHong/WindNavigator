from serial import Serial
import sys
import math

port = "/dev/ttyUSB0"
packetbyte = 90

def _HexArrToDec(data):
    littleEndianVal = 0
    for i in range(0,len(data)):
        littleEndianVal = littleEndianVal+(data[i]*(256**i))
    return littleEndianVal

def _CheckSum(data):
    try:
        ocs = data[4]
        cs = data[0]^data[1]^data[2]^data[3]
        for i in range(5, len(data)):
            cs ^=data[i]
            
        if(cs==ocs):
            return True
        else:
            return False
    except Exception as e:
        return False

def main():    
    data = []
    try:
        ydlidarX3p = Serial(port=port,
                            baudrate=115200
                            )
        
        raw_data = bytearray(ydlidarX3p.read(packetbyte))
        ydlidarX3p.close()
        
        i = 0
        
        while i < packetbyte:
            # 2바이트씩 묶여서 값이 오기 때문에 합해야함.
            # 즉, 하나의 값을 표현하기 위해 2Byte가 필요함
            temp = raw_data[i]
            temp += raw_data[i+1] << 8
            data.append(temp)  # 데이터 추가
            i += 2
        
        
            
    except Exception as e:
        ydlidarX3p.close()
        print("Access Denied")
        sys.exit()
        
    for i in range(len(raw_data)):
        print("{:=04X}".format(raw_data[i]), end=' ')    
    print()
    
    ocs = _HexArrToDec((raw_data[8], raw_data[9]))
    print("{:=05X}".format(ocs))
    
    cs = 0
    for i in range(4):
        cs ^= data[i]
    for i in range(5,len(data)):
        cs ^= data[i]
    
    print("{:=05X}".format(cs))
    print("{:=05X}".format(cs^ocs))
    
    
            
    """an_fsa = (data[2] >> 1) // 64   # 라이다 패킷의 현재 패킷의 시작 위치 각도
    an_lsa = (data[3] >> 1) // 64   # 현재 패킷의 끝 위치 각도
    lsn = data[1] >> 8      # 샘플링 포인트의 개수
    angle = []
    
    # 각도 차를 구해서 리스트에 담음
    for i in range(lsn):
        angle.append(((abs(an_fsa - an_lsa) // (lsn-1)) * i) + an_fsa)
    distance = []
    
    for data_index in range(5, len(data)):
        distance.append(data[data_index]//4)
    
    ang_correct = [] # 각도 보정을 진행하는데 담음
    
    for cor in range(len(distance)):
        # 먼저, 거리가 0이면 각도도 의미 없음. 0으로 보정
        if distance[cor] ==0:   
            ang_correct.append(0)
        else:
            cor_arg = math.degrees(math.atan(21.8 * (155.3-distance[cor])/(115.3 * distance[cor])))
            ang_correct.append(cor_arg)
    
    an_fsa = angle[0] + ang_correct[0]
    an_lsa = angle[len(distance) - 1] + ang_correct[len(distance) - 1]
    
    cor_angle = []
    i = 0
    for i in range(lsn):
        cor_angle.append(int((abs(an_fsa-an_lsa)//(lsn-1)) * i) + an_fsa)
        
    for idx in range(len(distance)):
        print("angle : {0} degrees, Distance: {1} mm".format(cor_angle[idx], distance[idx]))"""
        
if __name__ == "__main__":
    main()