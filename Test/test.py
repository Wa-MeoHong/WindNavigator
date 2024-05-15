from serial import Serial
import time
import math
from math import atan, pi, floor
import matplotlib.pyplot as plt

port = '/dev/ttyUSB0'
baudrate = 115200

def plot_lidar(distdict):
    x = [0 for i in range(360)]
    y = [0 for i in range(360)]
    for angle in range(0, 360):
        x[angle] = distdict[angle] * math.cos(math.radians(angle))
        y[angle] = distdict[angle] * math.sin(math.radians(angle))

    plt.show()
    plt.figure(1)
    plt.cla()
    plt.ylim(-5000, 5000)
    plt.xlim(-5000, 5000)
    plt.scatter(x, y, c='r', s=8)
    plt.pause(0.001)

def _CheckSum(data):
    try:
        ocs = _HexArrToDec((data[6],data[7]))
        LSN = data[1]
        cs = 0x55AA^_HexArrToDec((data[0],data[1]))^_HexArrToDec((data[2],data[3]))^_HexArrToDec((data[4],data[5]))
        for i in range(0,2*LSN,2):
            cs = cs^_HexArrToDec((data[8+i],data[8+i+1])) 
            
        if(cs==ocs):
            return True
        else:
            return False
    except Exception as e:
        return False
    
def _HexArrToDec(data):
    littleEndianVal = 0
    for i in range(0,len(data)):
        littleEndianVal = littleEndianVal+(data[i]*(256**i))
    return littleEndianVal

def _AngleCorr(dist):
    if dist==0:
        return 0
    else:
        return (atan(21.8*((155.3-dist)/(155.3*dist)))*(180/pi))
        
def _Calculate(d):
    ddict=[]
    LSN=d[1]
    Angle_fsa = ((_HexArrToDec((d[2],d[3]))>>1)/64.0)
    Angle_lsa = ((_HexArrToDec((d[4],d[5]))>>1)/64.0)
    if Angle_fsa<Angle_lsa:
        Angle_diff = Angle_lsa-Angle_fsa
    else:
        Angle_diff = 360+Angle_lsa-Angle_fsa
    for i in range(0,2*LSN,2):
        dist_i = _HexArrToDec((d[8+i],d[8+i+1]))/4
        Angle_i_tmp = ((Angle_diff/float(LSN))*(i/2))+Angle_fsa
        if Angle_i_tmp > 360:
            Angle_i = Angle_i_tmp-360
        elif Angle_i_tmp < 0:
            Angle_i = Angle_i_tmp+360
        else:
            Angle_i = Angle_i_tmp
        
        Angle_i = Angle_i +_AngleCorr(dist_i)
        ddict.append((dist_i,Angle_i))
    return ddict
 
def _Mean(data):
    length_of_data_without_zero = sum([i!=0 for i in data])
    if(len(data)>0 and length_of_data_without_zero!=0):
#        return int(sum(data)/len(data)) # original By ydlidar
        return float(sum(data)/length_of_data_without_zero) # modified for remove zero value
    return 0

def code(ser):
    data1 = ser.read(6000)
    data2=data1.split(b"\xaa\x55")[1:-1]
    
    distdict = {}
    for i in range(0,360):
        distdict.update({i:[]})
    for i,e in enumerate(data2):
        try:
            if(e[0]==0):
                if(_CheckSum(e)):
                    d = _Calculate(e)
                    for ele in d:
                        angle = floor(ele[1])
                        if(angle>=0 and angle<360):
                            distdict[angle].append(ele[0])
        except Exception as e:
            pass
    for i in distdict.keys():
        distdict[i]=_Mean(distdict[i])
    yield distdict
    
def main():
    # Open Serial
    ser = Serial(port=port,baudrate=baudrate)
    ser.isOpen()
    print("Connected!")
     
    # Scan start
    
    raw_data = bytearray(ser.read(80))
    ser.close()
    
    for i in range(80):
        print(raw_data[i],end=' ')
    
    """values = bytearray([int('a5', 16),int('60', 16)])
    ser.write(values)
        
    for i in range(10):
        angle_data = code(ser)
        plot_lidar(next(angle_data))
    
    # Scan End
    values = bytearray([int('a5', 16),int('65', 16)])
    ser.write(values)
    
    # Close Serial
    ser.close()    
"""
if __name__ == '__main__':
    main()
