#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

image_w = 320
image_h = 240
lower = []
upper = []
img =None

def image_callback(msg):
    global img, mage_w, image_h
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (image_w, image_h))


def range_str2list(str):
    str = str.split(',')
    return np.array([int(str[0]), int(str[1]), int(str[2])], np.uint8)


def do_mission():
    global img, lower, upper
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    cv2.imshow('img',img)
    cv2.imshow('mask',mask)
    cv2.waitKey(1)


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
    lower = rospy.get_param('color_range/color_example/lower_orange')
    upper = rospy.get_param('color_range/color_example/upper_orange')
    lower = range_str2list(lower)
    upper = range_str2list(upper)

    while not rospy.is_shutdown():
        if img is None:
            continue
        do_mission()
# rospy.spin()
