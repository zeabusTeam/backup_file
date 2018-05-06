#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image


image_w = 320
image_h = 240
lower = []
upper = []
color = []
mask_list = []
cnts = []
area_each_pole = []
img = None;hsv = None
task = ''
def image_callback(msg):
    global img, image_w, image_h,hsv
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (image_w, image_h))
    hsv = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2HSV)
def range_str2list(str):
    str = str.split(',')
    return np.array([int(str[0]), int(str[1]), int(str[2])], np.uint8)
def craete_empty_list():
    global mask_list,cnts
    cnts = [None]*len(color)
    mask_list = [None]*len(color)
    area_each_pole = [None]*len(color)

def compare_size(a1,a2):
    if a1/a2 > 1:
        print('left')
    elif a1/a2 == 0:
        print('stop')
    else:
        print('right')

def send_location():
    global cnts
    old_area = 0
    for i in range(len(mask_list)):
        ret,th = cv2.threshold(mask_list[i],127,255,0)
        im2,cnts[i],hi =cv2.findContours(th,1,2)
        for j in cnts[i]:
            center , (width, height), angle = cv2.minAreaRect(j)
            area = width * height
            x,y = center
            if area > 5000:
                right_top_corner = (x+width/2,y+height/2)
                right_bottom_corner =(x+width/2,y-height/2)
                left_bottom_corner = (x-width/2,y-height/2)
                left_top_corner = (x-width/2,y+height/2)
                old_area = area
    compare_size(area_each_pole[0],area_each_pole[1])
def get_color():
    global lower,upper
    for i in color:
        tmp_lower = rospy.get_param('color_range/color_example/lower_'+i);tmp_upper = rospy.get_param('color_range/color_example/upper_'+i)
        tmp_lower = range_str2list(tmp_lower);tmp_upper=range_str2list(tmp_upper)
        upper.append(tmp_upper);lower.append(tmp_lower)
    craete_empty_list()
def showmask():
    global mask_list
    cv2.imshow('original',img)
    for i in range(len(color)):
        mask = cv2.inRange(hsv,lower[i],upper[i])
        mask_list[i] = mask
        cv2.imshow(color[i],mask)
    cv2.waitKey(1)
if __name__ == '__main__':
    rospy.init_node('vision_mission',anonymous = True)
    color = ['orange','green','black']
    task = 'gate'
    get_color()
    nodeName = 'mission'
    cameraTopic = rospy.get_param(nodeName + '/cameraTopic',
                                  '/leftcam_bottom/image_raw/compressed')
    print('TOPIC: ' + str(cameraTopic))
    rospy.Subscriber(cameraTopic, CompressedImage, image_callback)
    while not rospy.is_shutdown():
        if img is None:
            continue
        showmask()
        send_location()
