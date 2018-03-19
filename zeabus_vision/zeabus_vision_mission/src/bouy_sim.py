#!/usr/bin/env python
import sys
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
sys.path.append(
    '/home/zeabus/catkin_ws/src/src_code/zeabus_vision/zeabus_vision_main/src/')
from vision_lib import *
import constant as CONST
import math
from std_msgs.msg import String
from zeabus_vision_srv_msg.srv import *
from zeabus_vision_srv_msg.msg import *

img = None
img_gray = None
hsv = None
# width = int(1152 / 3)
height = int(870 / 3)
width = None
height = None
resultMemory = []
getMemoryStatus = True
xMemory = 0.0
yMemory = 0.0
radiusOverlay = 15
ratioAreaCir = 0.5
minAreaCir = 30
mode = 1
resImg = None
trackRadius = 50
req = None
font = cv2.FONT_HERSHEY_SIMPLEX
# mode = 1  r y g
# mode = 2 g y r


def image_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img1 = cv2.imdecode(arr, 1)
    h, w, ch = img1.shape
    width = w / 2
    height = h / 2
    img = cv2.resize(img1, (width, height))


def mission_callback(msg):
    global req
    req = msg.req.data
    print('request: ') + str(req)
    return find_bouy(req)

# def velocity_callback():


def process_mask(imgBIN):
    kernelFrame = get_kernel('plus', (3, 3))

    kernelRow = get_kernel('rect', (5, 3))
    kernelCol = get_kernel('rect', (3, 5))

    resDilateRow = dilate(imgBIN, kernelRow)
    resDilateCol = dilate(imgBIN, kernelCol)
    resDilate = resDilateRow + resDilateCol
    resErode1 = erode(resDilate, kernelFrame)
    resErode = erode(resErode1, kernelFrame)
    return resErode


def print_result(msg):
    print '<---------- ' + str(msg) + ' ---------->'


def radius_distance(preX, curX, preY, curY):
    return (preX - curX)**2 + (preY - curY)**2


def circle_area(radius):
    return math.pi * (radius**2)


def identify_circle(detectArea, realArea):
    global ratioAreaCir, minAreaCir
    if ratio_area(detectArea, realArea) <= ratioAreaCir:
        return False
    elif detectArea <= minAreaCir:
        return False
    return True


def ratio_area(detectArea, realArea):
    return detectArea / realArea


def offset_centroid(cx, cy):
    global width, height
    offsetW = width / 2.0
    offsetH = height / 2.0
    cx = -((cx - offsetW) / offsetW)
    cy = (offsetH - cy) / offsetH
    return cx, cy


def not_found():
    return -999, -999, -999, False


def find_all_bouy(contours):
    global radiusOverlay, minAreaCir, mode, resultMemory, resImg, font

    print_result('Find All')

    statusFilter = False
    resultCir = []
    count = 0
    for c in contours:
        # cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
        area = cv2.contourArea(c)
        (x, y), r = cv2.minEnclosingCircle(c)
        areaCir = circle_area(r)
        cirPerScreen = ratio_area(areaCir, (width * height))

        if not identify_circle(area, areaCir):
            continue
        if statusFilter and radius_distance(preX, x, preY, y) <= radiusOverlay**2:
            continue

        preX = x
        preY = y
        statusFilter = True
        resultCir.append([x, y, cirPerScreen])
        cv2.circle(resImg, (int(x), int(y)), int(r), (255, 255, 255), 3)
        cv2.putText(resImg, 'Area: %.2f %.2f %d ' %
                    (cirPerScreen,
                     ratio_area(area, areaCir), (area)),
                    (int(x), int(y)), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    # min to max
    resultCir = sorted(resultCir, key=lambda l: l[0], reverse=False)
    resultR = []
    resultG = []
    resultY = []
    resultMemory = []
    for i in xrange(min(3, len(resultCir))):
        x, y, area = resultCir[i]
        if i == 0:
            if mode == 1:
                resultR = [x, y, area]
                color = CONST.COLOR_RED
            else:
                resultG = [x, y, area]
                color = CONST.COLOR_GREEN
            cv2.circle(resImg, (int(x), int(y)), 6, color, -1)
        elif i == 1:
            resultY = [x, y, area]
            color = CONST.COLOR_YELLOW
            cv2.circle(resImg, (int(x), int(y)), 6, color, -1)
        else:
            if mode == 2:
                resultR = [x, y, area]
                color = CONST.COLOR_GREEN
            else:
                resultG = [x, y, area]
                color = CONST.COLOR_GREEN
            cv2.circle(resImg, (int(x), int(y)), 6, color, -1)
        publish_result(resImg, 'bgr', '/bouy_result')
        count += 1
    if count == 3:
        resultMemory.append([resultR])
        resultMemory.append([resultY])
        resultMemory.append([resultG])
    return count


def find_yellow_bouy(contours):
    global radiusOverlay, minAreaCir, mode, resultMemory, resImg, font
    statusFilter = False
    resultCir = []
    count = 0
    for c in contours:
        # cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
        area = cv2.contourArea(c)
        (x, y), r = cv2.minEnclosingCircle(c)
        areaCir = circle_area(r)
        cirPerScreen = ratio_area(areaCir, (width * height))

        if not identify_circle(area, areaCir):
            continue
        if statusFilter and radius_distance(preX, x, preY, y) <= radiusOverlay**2:
            continue
        resultCir.append([x, y, cirPerScreen])
        preX = x
        preY = y
        statusFilter = True

        cv2.circle(resImg, (int(x), int(y)), int(r + 10), (125, 125, 125), 3)
        cv2.putText(resImg, 'Area: %.2f %.2f %d ' %
                    (cirPerScreen, ratio_area(area, areaCir), areaCir),
                    (int(x), int(y)), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    if len(resultCir) > 0:
        resultCir = sorted(resultCir, key=lambda l: l[2], reverse=True)
        resultCir = resultCir[0]
        x = resultCir[0]
        y = resultCir[1]
        cv2.circle(resImg, ((int(x)),
                            int(y + 10)), 5, CONST.COLOR_ORANGE, -1)
        publish_result(resImg, 'bgr', '/bouy_result')
        x, y = offset_centroid(x, y)
        area = resultCir[2]
        appear = True
        return x, y, area, appear
    else:
        return not_found()


def get_bouy_position():
    global getMemoryStatus, resultMemory, req, mode, xMemory, yMemory

    if getMemoryStatus:
        if mode == 1:
            if req == 'r':
                resultMemory = resultMemory[0]
            elif req == 'y':
                resultMemory = resultMemory[1]
            elif req == 'g':
                resultMemory = resultMemory[2]
        else:
            if req == 'g':
                resultMemory = resultMemory[0]
            elif req == 'y':
                resultMemory = resultMemory[1]
            elif req == 'r':
                resultMemory = resultMemory[2]
        resultMemory = resultMemory[0]
        xMemory = resultMemory[0]
        yMemory = resultMemory[1]
        getMemoryStatus = False


def find_one_bouy(contours):
    global radiusOverlay, minAreaCir, mode, resultMemory, resImg, font, xMemory, yMemory, trackRadius

    get_bouy_position()
    statusFilter = False
    resultCir = []
    count = 0
    print_result('Find One Bouy')

    for c in contours:
        area = cv2.contourArea(c)
        cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
        (x, y), r = cv2.minEnclosingCircle(c)
        areaCir = circle_area(r)
        cirPerScreen = ratio_area(areaCir, (width * height))

        if not identify_circle(area, areaCir):
            continue

        if radius_distance(x, xMemory, y, yMemory) >= trackRadius**2:
            continue
        resultCir.append([x, y, cirPerScreen])
        cv2.circle(resImg, (int(x), int(y)), int(r), (255, 255, 255), 2)
        cv2.putText(resImg, 'Area: %.2f %.2f %d ' %
                    (cirPerScreen, ratio_area(area, areaCir), areaCir),
                    (int(x), int(y)), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(resImg, (int(x), int(y)), int(
            r + trackRadius), (225, 0, 0), 3)
        publish_result(resImg, 'bgr', '/bouy_result')
        resultCir.append([x, y, cirPerScreen])

    if len(resultCir) > 0:
        resultCir = sorted(resultCir, key=lambda l: l[2], reverse=False)
        x = resultCir[-1][0]
        y = resultCir[-1][1]
        cv2.circle(resImg, ((int(x)),
                            int(y)), 5, (0, 255, 255), -1)
        publish_result(resImg, 'bgr', '/bouy_result')
        xMemory = x
        yMemory = y
        x, y = offset_centroid(x, y)
        area = resultCir[-1][2]
        appear = True
        print xMemory, yMemory
        return x, y, area, appear
    else:
        return not_found()


def find_bouy(req):
    global img, width, height, resultMemory, getMemoryStatus, xMemory, yMemory, mode, trackRadius, font, resImg

    m = vision_msg_bouy()

    lowerY, upperY = get_color('yellow', 'top', 'bouy_sim')
    lowerR, upperR = get_color('red', 'top', 'bouy_sim')
    lowerYY, upperYY = get_color('orange', 'top', 'bouy_sim')

    while img is None:
        print('img is none in loop')
        continue

    resPreprocess = preprocess_bouy(img)
    resHSV = cv2.cvtColor(resPreprocess.copy(), cv2.COLOR_BGR2HSV)
    resImg = resPreprocess.copy()

    resY = cv2.inRange(resHSV, lowerY, upperY)
    processMaskY = process_mask(resY)
    resR = cv2.inRange(resHSV, lowerR, upperR)
    processMaskR = process_mask(resR)
    resYY = cv2.inRange(resHSV, lowerYY, upperYY)
    processMaskYY = process_mask(resYY)

    mask = cv2.bitwise_or(processMaskR, processMaskR, mask=processMaskY)
    maskInv = np.invert(mask)
    maskCrop = crop_gray(maskInv, 10, 10)
    kernelFrame = get_kernel('plus', (5, 5))
    maskErode = erode(maskCrop, kernelFrame)

    _, thAllBouy = cv2.threshold(maskErode.copy(), 20, 255, 0)
    _, contoursAllBouy, _ = cv2.findContours(
        thAllBouy.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    _, thYellowBouy = cv2.threshold(processMaskYY.copy(), 20, 255, 0)
    _, contoursYellowBouy, _ = cv2.findContours(
        thYellowBouy.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if req == 'a':
        count = find_all_bouy(contoursAllBouy)
        x, y, area, appear = find_yellow_bouy(contoursYellowBouy)
        m.num = count
        m.cx = x
        m.cy = y
        m.area = area
        m.appear = appear
        getMemoryStatus = True

    elif req == 'o':
        x, y, area, appear = find_yellow_bouy(contoursYellowBouy)
        m.num = int(appear)
        m.cx = x
        m.cy = y
        m.area = area
        m.appear = appear
    else:
        x, y, area, appear = find_one_bouy(contoursAllBouy)
        m.num = int(appear)
        m.cx = x
        m.cy = y
        m.area = area
        m.appear = appear

    publish_result(resImg, 'bgr', '/bouy_result')
    publish_result(mask, 'gray', '/bouy_mask')
    publish_result(thAllBouy, 'gray', '/bouy_mask_inv')

    print m
    return m

if __name__ == '__main__':
    rospy.init_node('vision_squid', anonymous=True)
    # topic = "/top/center/image_rect_color/compressed"
    topic = "/syrena/front_cam/image_raw/compressed"
    rospy.Subscriber(topic, CompressedImage, image_callback)
    # find_bouy()
    rospy.Service('vision_bouy', vision_srv_bouy(), mission_callback)
    rospy.spin()
