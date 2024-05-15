import threading
from Lidar import YDLidarX3pro
import matplotlib.pyplot as plt
import math
import time

def draw():
    global is_plot
    while is_plot:
        plt.figure(1)
        plt.cla()
        plt.ylim(-5000, 5000)
        plt.xlim(-5000, 5000)
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
    port = '/dev/ttyUSB0'
    obj = YDLidarX3pro(port=port)
    threading.Thread(target=draw).start()
    if(obj.Connect()):
        print(obj.GetDeviceInfo())
        gen = obj.StartScanning()
        t = time.time()
        # while (time.time() - t) < 30: #scan for 30 seconds
        while True:
            data = next(gen)
            for angle in range(0, 360):
                x[angle] = data[angle] * math.cos(math.radians(angle))
                y[angle] = data[angle] * math.sin(math.radians(angle))
        is_plot = False
        obj.StopScanning()
        obj.Disconnect()

    else:
        print("Error connecting to device")
        
if __name__ == "__main__":
    main()