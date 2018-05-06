#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

image_w = 320
image_h = 240
lower = [];mask_list=[]
upper = []
img =None

def image_callback(msg):
    global img, mage_w, image_h
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (image_w, image_h))
def create(c):
    for i in c:
        cv2.namedWindow(c)

def range_str2list(str):
    str = str.split(',')
    return np.array([int(str[0]), int(str[1]), int(str[2])], np.uint8)

def do_mission():
    global img, lower, upper,hsv,mask_list
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask_list.append(mask)
    cv2.imshow('img',img)
    cv2.imshow('mask',mask)
    cv2.waitKey(1)
    _,th = cv2.threshold(mask.copy(),127,255,0)
    im2,cnts,hierarchy  = cv2.findContours(th, 1, 2)
    for i in cnts:
        center , (width, height), angle = cv2.minAreaRect(i)
        area = width * height
        x,y = center
        if area > 5000:
            right_top_corner = (x+width/2,y+height/2)
            right_bottom_corner =(x+width/2,y-height/2)
            left_bottom_corner = (x-width/2,y-height/2)
            left_top_corner = (x-width/2,y+height/2)
if __name__ == '__main__':
    rospy.init_node('vision_example', anonymous=True)
    # Nodename in launch file u can see in node tag <node>, u will see name=.......
    nodeName = 'mission'
    # get_param will get the value from node/param_name
    # In launch u can see param tag <para> ,.................
    cameraTopic = rospy.get_param(nodeName + '/cameraTopic',
                                  '/leftcam_bottom/image_raw/compressed')
    print('TOPIC: ' + str(cameraTopic))
    rospy.Subscriber(cameraTopic, CompressedImage, image_callback)
    # Please open color_example.yaml while read this method
    # Warning: get_param is call rospy.get_param again
    # get param from namespace/first level tag (First line in yaml file)/second level tag (Second and the next line)
    num = int(raw_input('enter number of color: '))
    if num ==1:
        color = raw_input('enter color: ')
        lower = rospy.get_param('color_range/color_example/lower_'+color)
        upper = rospy.get_param('color_range/color_example/upper_'+color)
        lower = range_str2list(lower)
        upper = range_str2list(upper)
    elif num>1:
        c = []
        for i in range(num):
            tmp = raw_input('color: ')
            c.append(tmp)
    while not rospy.is_shutdown():
        if img is None:
            continue
        if num == 1:
            do_mission()
        elif num>1:
            for i in c:
                lower = rospy.get_param('color_range/color_example/lower_'+i)
                upper = rospy.get_param('color_range/color_example/upper_'+i)
                lower = range_str2list(lower)
                upper = range_str2list(upper)
                do_mission()
    # rospy.spin()
