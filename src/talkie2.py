import serial
import time
import numpy as np
alt = 0
Xrot0 = 360
Yrot0 = 360
t = 0
PI = 3.14159265359792
ser = serial.Serial('/dev/cu.usbserial-D3092QGQ',baudrate=115200)
data = bytearray()
while(1):
    t+=1
    data = bytearray()
    data.append(0x20)
    data.append(0x20)
    data.append(0x00)
    data.append(0x0d)
    data.append(0x0a)
    ser.write(data)
    print(f'({t}) Bin out: ', end=' ')
    for byte in data:
        print(f'{byte:0>8b}', end=' ')
    print()
    time.sleep(0.5)


