#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
from sensor_msgs.msg import CompressedImage
import sys
sys.path.append ('/home/zeabus/catkin_ws/src/src_code/zeabus_vision/zeabus_vision_main/src/')
from vision_lib import *
import constant as const
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default

img = None
width = const.IMAGE_BOTTOM_WIDTH
height = const.IMAGE_BOTTOM_HEIGHT

lower_orange, upper_orange = get_color('orange', 'bottom', 'path')

def find_path():
    global img, width, height
    t = True
    f = False
    cnt = None
    res = vision_msg_default()

    while img is None:
        print("img: None")
        rospy.sleep(0.01)
    print('upper_or', upper_orange)
    print('lower_or', lower_orange)
    im = img.copy()
    offsetW = width/2
    offsetH = height/2
    area = -999
    max1 = -999
    cx = -999
    cy = -999
    w = -999
    h = -999
    angle = -999
    box = None

    imStretching = stretching(im)
    imgForDraw = img.copy()
    imgBlur = cv2.GaussianBlur(im, (3,3), 0)

    bgr = preprocess_path(im)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    imgray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    imgOrange = cv2.inRange(hsv, lower_orange, upper_orange)
    imgOrange = close(imgOrange, get_kernel())

    ret, thresh = cv2.threshold(imgray, 200, 255, 0)
    _, contours, hierarchy = cv2.findContours(imgOrange.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    realArea = 0
    ratioArea = 0
    # area = 1

    for c in contours:
        M = cv2.moments(c)
        rect = (x,y),(ww,hh),angle1 =cv2.minAreaRect(c)
        area = ww*hh

        realArea = cv2.contourArea(c)
        print('========================')
        print('area', area)
        print('realarea', realArea)
        print('========================')


        if area != 0:
            ratioArea = (realArea/area)*100 
        
        if area < 500:
            continue
        if hh == 0 :
            continue
        if ww > hh:
            tmp = hh
            hh = ww
            ww = tmp
        diff = (ww/hh)
        epsilon = 0.1*cv2.arcLength(c, t)
        approx = cv2.approxPolyDP(c, epsilon,t)
        
        if area > max1:
            max1 = area
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            angle = 90-Oreintation(M)[0]*180/math.pi
            cx = x
            cy = y
            w = ww
            h = hh

    cv2.circle(imgForDraw,(int(cx), int(cy)), 5, (0, 0, 255), -1)
    cv2.drawContours(imgForDraw,[box], -1,(0,255,255),3)
    publish_result(imgForDraw, 'bgr', 'debug_path')
    publish_result(imgOrange, 'gray', 'inRange')
    xx = (cx-offsetW)/offsetW
    yy = (offsetH-cy)/offsetH
    res.x = yy
    res.y = -xx
    res.area = max1
    res.appear = True
    res.angle = angle
    print('max1',max1)
    if max1<500:
        xx = -999
        yy = -999
        res.appear = False
        res.x = -999
        res.y = -999
        res.area = -999
        res.angle = -999
    print('x', yy)
    print('y', -xx)
    print('max',max1)
    print('angle', angle)
    return res
    
def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (width, height))
 


def mission_callback(msg):
    print('mission_callback')
    return find_path()

if __name__ == '__main__':
    rospy.init_node('findPath1')
    topic = '/bottom/left/image_raw/compressed'
    bot = '/bottom/left/image_raw/compressed'
    rospy.Subscriber(bot, CompressedImage, img_callback)
    rospy.Service('vision', vision_srv_default(), mission_callback)
    rospy.spin()