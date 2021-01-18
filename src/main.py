###########################################################################################################################
#
#    Main method for Autonomous Quadcopter
#
#    Author: Daniel Hoven
#
#    Date Opened: 1/8/21
#
#    Remote: https://github.com/Dhoven23/CapstoneProject
#
#--------------------------------------------------------------------------------------------------------------------------

import cv2
import numpy as np
import serial
import ComputerVisionAlgorithms as CVA
import time
#/////////////
# Initialize sensors, motors, and telemetry
#/////////////

def TeensyConnect():
    teensy = serial.Serial('/dev/cu.usbmodem81837901')
    if teensy.isOpen():
        teensy.close()
    teensy.open()
    time.sleep(0.5)
    for i in range (0,5): teensy.write(bytes(b'N'))
    time.sleep(0.5)
    if (teensy.inWaiting() > 0):
        reply = teensy.readline()
        reply = str(reply)
        print(reply)


def main():

    print("Entry Point Formed!")
    TeensyConnect()

    CVA.test()

if __name__ == '__main__':
    main()

