import serial
import time
import numpy as np
alt = 0
Xrot0 = 360
Yrot0 = 360
t = 0.00
PI = 3.14159265359792
ser = serial.Serial('/dev/cu.usbmodem83805901')
data = bytearray()
while(1):
    t+=0.01
    data = bytearray()
    Xrot = int(np.sin(t) + Xrot0)
    Yrot = int(np.cos(t) + Yrot0)
    alt=10
    data.append(0x59)
    data.append(0x59)
    data.append(alt&255)
    data.append((alt>>8)&255)
    data.append(Xrot&255)
    data.append((Xrot>>8)&255)
    data.append(Yrot&255)
    data.append((Yrot>>8)&255)
    data.append((alt+Xrot+Yrot)&255)
    print(alt,Xrot,Yrot)
    ser.write(data)
    time.sleep(0.05)


