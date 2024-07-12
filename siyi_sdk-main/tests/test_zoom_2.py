"""
@file test_center_gimbal.py
@Description: This is a test script for using the SIYI SDK Python implementation to adjust zoom level
@Author: Mohamed Abdelkader
@Contact: mohamedashraf123@gmail.com
All rights reserved 2022
"""

import sys
import os
from time import sleep
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

def test():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)

    if not cam.connect():
        print("No connection ")
        exit(1)

    while cam.getZoomLevel() < 3.0:
        val = cam.requestZoomIn()
        
    print(f"Zoom level: {cam.getZoomLevel()}")

    val = cam.requestZoomHold()

    # cam.setGimbalRotation(10,20, err_thresh=30, kp=1)
    # cam.setGimbalRotation(-10,-90, err_thresh=1, kp=4)
    cam.setGimbalRotation2(yaw=0, pitch=-90, err_thresh=1, kp=4, kd=0.1)
    

#    while True:
#        print(f"Sending autofocus request...")
#        val = cam.requestAutoFocus()
#        print(val)
#        sleep(2)

    cam.disconnect()

if __name__ == "__main__":
    test()
