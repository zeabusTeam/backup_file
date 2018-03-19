#!/usr/bin/env python
import sys
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
sys.path.append(
    '/home/zeabus/catkin_ws/src/src_code/zeabus_vision/zeabus_vision_main/src/')
from vision_lib import *
import math
from std_msgs.msg import String
from zeabus_vision_srv_msg.srv import *
from zeabus_vision_srv_msg.msg import *

img = None
img_gray = None
hsv = None
width = int(1152 / 3)
height = int(870 / 3)
v = [0, 0]


def image_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (width, height))


def velocity_callback(msg):
    global v
    v[0] = msg.linear.y
    v[1] = msg.linear.z
    print v


def mission_callback(msg):
    print('mission_callback')
    req = msg.req.data
    print('request: ') + str(req)
    return find_squid(req)


def not_found():
    m = vision_msg_default()
    m.x = 0
    m.y = 0
    m.area = 0
    m.angle = 0
    m.appear = False
    return m


def find_squid(req):
    global img, width, height

    lowerY, upperY = get_color('yellow', 'top', 'squid')
    lowerW, upperW = get_color('white', 'top', 'squid')
    lowerR, upperR = get_color('red', 'top', 'squid')

    font = cv2.FONT_HERSHEY_SIMPLEX

    kernelFrame = get_kernel('plus', (5, 5))
    kernelRow = get_kernel('rect', (3, 1))
    kernelCol = get_kernel('rect', (1, 3))

    m = vision_msg_default()
    m.appear = False
    m.angle = 0
    m.area = 0
    m.x = 0
    m.y = 0
    minArea = 500
    offsetW = width / 2.0
    offsetH = height / 2.0

    while img is None:
        print('img is none in loop')

    resPreprocess = preprocess_squid(img)
    resHSV = cv2.cvtColor(resPreprocess.copy(), cv2.COLOR_BGR2HSV)
    resImg = resPreprocess.copy()
    resultFourCir = []
    resultTwoCir = []

    maskW = cv2.inRange(resHSV, lowerW, upperW)
    maskR = cv2.inRange(resHSV, lowerR, upperR)

    maskYPre = cv2.inRange(resHSV, lowerY, upperY)
    maskY = dilate(maskYPre, kernelFrame)
    maskYBlur = cv2.medianBlur(maskYPre, 3)

    mask1 = cv2.subtract(maskYBlur, maskW)
    mask = cv2.subtract(mask1, maskR)
    maskInv = np.invert(mask)

    _, th = cv2.threshold(maskInv, 20, 255, 0)

    edges = cv2.Canny(th.copy(), 100, 200)
    kernelFrame = get_kernel('plus', (5, 5))
    edgesDilateInv = np.invert(dilate(edges, kernelFrame))

    thEdge = cv2.bitwise_and(edgesDilateInv, edgesDilateInv, mask=th)

    _, contours, _ = cv2.findContours(
        thEdge.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    statusFilter = False
    for c in contours:
        area = cv2.contourArea(c)
        cv2.drawContours(resImg, [c], -1, (222, 2, 222), 2)
        if area < minArea:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        areaCir = (math.pi * (r**2))
        if area / areaCir <= 0.65:
            continue
        if statusFilter and (x - x_before)**2 + (y - y_before)**2 <= 10**2:
            continue
        cv2.circle(resImg, (int(x), int(y)), int(r), (255, 255, 0), 3)
        cv2.circle(resImg, (int(x), int(y)), 2, (0, 255, 0), -1)
        cv2.putText(resImg, 'Area: %.2f %d' %
                    (((areaCir / (width * height))), (areaCir)),
                    (int(x), int(y)), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        resultFourCir.append([x, y, areaCir / (width * height)])
        x_before = x
        y_before = y
        statusFilter = True

    if len(resultFourCir) > 0:
        resultFourCir = sorted(resultFourCir, key=lambda l: l[0], reverse=True)
        # if abs(resultFourCir[0][0] - resultFourCir[-1][0]) <= 20:
        #     m = not_found()
        cutterX = (resultFourCir[0][0] + resultFourCir[-1][0]) / 2.0
        cv2.line(resImg, (int(cutterX - 10), 0),
                 (int(cutterX - 10), height), (212, 123, 132), 5)
        for res in resultFourCir:
            x = res[0]
            if x <= cutterX - 10:  # statusFilter and
                continue
            resultTwoCir.append(res)

    if len(resultTwoCir) > 0:
        resultTwoCir = sorted(resultTwoCir, key=lambda l: l[2], reverse=True)
        m.appear = True
        if req == 's':
            m.x = resultTwoCir[-1][0]
            m.y = resultTwoCir[-1][1]
            m.area = resultTwoCir[-1][2]
        elif req == 'b':
            m.x = resultTwoCir[0][0]
            m.y = resultTwoCir[0][1]
            m.area = resultTwoCir[0][2]

        cv2.circle(resImg, ((int(m.x)), int(m.y)), 6, (255, 255, 255), -1)

        m.x = -((m.x - offsetW) / offsetW)
        m.y = (offsetH - m.y) / offsetH

    else:
        m = not_found()

    print m

    publish_result(th, 'gray', '/squid_result_gray')
    publish_result(thEdge, 'gray', '/squid_result_range_inv')
    publish_result(resImg, 'bgr', '/squid_result')
    return m

if __name__ == '__main__':
    rospy.init_node('vision_squid', anonymous=True)
    imageTopic = "/sy"
    # imageTopic = "/top/center/image_rect_color/compressed"
    # velocityTopic = "/syrena/cmd_vel"
    velocityTopic = "/zeabus/cmd_vel"
    rospy.Subscriber(imageTopic, CompressedImage, image_callback)
    rospy.Subscriber(velocityTopic, Twist, velocity_callback)
    rospy.Service('vision_squid', vision_srv_default(), mission_callback)
    rospy.spin()
