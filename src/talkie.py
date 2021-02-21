import serial
import Wxwidgets as wx


import time
import numpy as np
root = tk.Tk()
#PI = 3.14159265359792
ser = serial.Serial('/dev/cu.usbmodem88151801')

def send(word):
    data = bytearray()
    word = list(word)
    Dcode = ord(word[0])
    Ncode = ''.join(word[1:])
    data.append(0x20)
    data.append(0x20)
    data.append(Dcode)
    t1 = (int(Ncode)&255)
    t2 = ((int(Ncode)>>8)&255)
    data.append(t1)
    data.append(t2)
    ser.write(data)

def forward():
    send('w1')





