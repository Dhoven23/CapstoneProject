import serial
import time
ser = serial.Serial('/dev/cu.usbserial-D3092QGQ')
count = 0
while(1):
    data = 0
    count+=1
    data = ser.read(5)
    print(f'({count}) Bin in: ', end = ' ')
    for byte in data:
        print(f'{byte:0>8b}', end = ' ')
    print()
