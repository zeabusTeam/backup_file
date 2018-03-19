#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
import sys
sys.path.append ('/home/zeabus/catkin_ws/src/src_code/zeabus_vision/zeabus_vision_main/src/')
from vision_lib import *
from sensor_msgs.msg import CompressedImage
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default

img = None
height = None
width = None

def find_table():
    global img, width, height

    minArea = 200

    lower_bg, upper_bg = get_color('black', 'bottom', 'table')
    lower_red, upper_red = get_color('red', 'bottom', 'table')
    lower_orange, upper_orange = get_color('orange', 'bottom', 'table')
    lower_blue, upper_blue = get_color('white', 'bottom', 'table')
    lower_green, upper_green = get_color('yellow', 'bottom', 'table')
    while not rospy.is_shutdown():
        while img is None:
            print('None img') 

        image = img.copy()
        imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        imageForDraw = img.copy()
        rectPlate = np.zeros((height, width))

        bgr = preprocess_table(image)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        bg = cv2.inRange(hsv, lower_bg, upper_bg)

        _, bg = cv2.threshold(bg, 20, 255, cv2.THRESH_BINARY_INV)
        # bg = np.invert(bg)
        bg = open_morph(bg, get_kernel())
        bg = close(bg, get_kernel())

        orange = cv2.inRange(hsv, lower_orange, upper_orange)
        blue = cv2.inRange(hsv, lower_blue, upper_blue)
        green = cv2.inRange(hsv, lower_green, upper_green)
        red = cv2.inRange(hsv, lower_red, upper_red)

        _, rectContours, _ = cv2.findContours(bg.copy(),
                                            cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)

        _, redContours, hierarchy = cv2.findContours(red.copy(), 
                                            cv2.RETR_EXTERNAL, 
                                            cv2.CHAIN_APPROX_SIMPLE)


        maxRed = 0
        angleRed = -999

        for c in redContours:
            M = cv2.moments(c)
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            realArea = cv2.contourArea(c)
            if realArea < minArea:
                continue
            if maxRed < area:
                maxRed = area
                boxRed = cv2.boxPoints(rect)
                boxRed = np.int0(boxRed)
                xRed = x
                yRed = y
                angleRed = 90-Oreintation(M)[0]*180/math.pi
        if maxRed != 0:
            cv2.drawContours(imageForDraw, [boxRed], -1, (0,0,255), 2)
            # appearRed = True
            print('angleRed', angleRed)

        
        dst = np.zeros((width, height))
        box = np.zeros((width, height))
        maxArea = 0
        angle1 = 0
        maxRect = (0,0), (0,0), 0
        angle = 0
        temp = -999
        for c in rectContours:
            M = cv2.moments(c)
            rect = (x,y), (ww, hh), angle1 = cv2.minAreaRect(c)
            temp = angle1
            # rect = (x,y), (ww,hh), 0
            area = ww*hh
            realArea = cv2.contourArea(c)
            # angle = 90-Oreintation(M)[0]*180/math.pi 
            if realArea < 10000:
                continue

            print('realArea',realArea)
            print('angle',angle)
            if maxArea < area:
                maxArea = area
                maxRect = rect
                angle = -angle1
                box = cv2.boxPoints(rect)
                box = np.int0(box)
        if maxArea != 0:
            print('temp', temp)
            cv2.drawContours(rectPlate, [box], -1, (255,255,255),2)
            cv2.drawContours(imageForDraw,[box],0,(0,0,255),2)
        
        M1 = cv2.getRotationMatrix2D((width/2,height/2),angle,1)
        dst = cv2.warpAffine(rectPlate,M1,(width,height))
        # dst = cv2.resize(dst, (width, height))

        cv2.imshow('red', red)
        cv2.imshow('imageForDraw', imageForDraw)
        cv2.waitKey(30)

def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 384))

    height, width, _ = img.shape
    

if __name__ == '__main__':
    rospy.init_node('findBin')
    bag = '/leftcam_bottom/image_raw/compressed'
    topic = '/bottom/left/image_raw/compressed'
    bot = '/bottom/left/image_raw/compressed'
    rospy.Subscriber(bot, CompressedImage, img_callback)
    find_table()
