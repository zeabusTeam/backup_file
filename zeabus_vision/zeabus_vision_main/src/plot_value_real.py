#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from scipy import stats
from sensor_msgs.msg import CompressedImage, PointCloud
from std_msgs.msg import Float32
import math
import dynamic_reconfigure.client
import time
import statistics
from matplotlib import pyplot as plt
from vision_lib import *

img = None
hsv = None

width = 480
height = 320


def callback(msg):
    global img, hsv, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (width, height))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def plot_value():
    global hsv, img, width, height
    cv2.namedWindow('image', flags=cv2.WINDOW_NORMAL)
    cv2.moveWindow('image', 20, 20)
    cv2.resizeWindow('image', width, height)
    # cv2.createTrackbar('v', 'image', 0, 255, nothing)
    plt.ion()
    while not rospy.is_shutdown():

        if hsv is None:
            rospy.sleep(0.01)
            print 'image is none'
            continue

        h, s, v = cv2.split(hsv)
        vOneD = v.ravel()
        # sOneD = s.ravel()

        cv2.imshow('image', img)
        cv2.imshow('hsv', hsv)
        plt.hist(vOneD, 256, [0, 256])
        plt.pause(0.01)
        plt.cla()
        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break

        # rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('adjust_exposure_time_bottom')
    # topic = '/top/center/image_rect_color/compressed'
    topic = '/bottom/left/image_raw/compressed'
    node = 'ueye_cam_nodelet_bottom_left/'
    rospy.Subscriber(topic, CompressedImage, callback)
    plot_value()
