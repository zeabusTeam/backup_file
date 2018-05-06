#!/bin/bash

#Setup camera

CAM0=$(readlink -f /dev/usbcamera/logitech_025039A0)

#uvcdynctrl --device=$CAM0 --set='Focus, Auto' 0
#uvcdynctrl --device=$CAM0 --set='Focus (absolute)' 0
#uvcdynctrl --device=$CAM1 --set='Focus, Auto' 0
#uvcdynctrl --device=$CAM1 --set='Focus (absolute)' 0

uvcdynctrl --device=$CAM0 --set='White Balance Temperature, Auto' 0
uvcdynctrl --device=$CAM0 --set='White Balance Temperature' 5000
#uvcdynctrl --device=$CAM0 --set='White Balance Temperature' 9000
#uvcdynctrl --device=$CAM0 --set='White Balance Temperature' 8000
uvcdynctrl --device=$CAM0 --set='Exposure, Auto' 1
uvcdynctrl --device=$CAM0 --set='Exposure (Absolute)' 100


#power line 
uvcdynctrl --device=$CAM0 --set='Power Line Frequency' 1
#uvcdynctrl --device=$CAM1 --set='Power Line Frequency' 1

#add more command
