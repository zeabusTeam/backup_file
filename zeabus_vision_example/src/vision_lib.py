#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import statistics
import operator
image = None

def preprocessing_gate(img_bgr):
    print_result('preprocessing_gate')
    b, g, r = cv2.split(img_bgr)
    r.fill(255)
    img_bgr = cv2.merge((b,g,r))
    blur = cv2.medianBlur(img_bgr,5)
    img_clahe = clahe(img_bgr)
    img_equ = equalization_bgr(blur)

    return img_equ

def preprocessing_flare(img_bgr):
    
    blur = cv2.medianBlur(img_bgr,3)
    img_equ = equalization_bgr(blur)
    return img_equ


def preprocessing_mat(img_bgr):
    print_result('preprocessing_drum')
    b, g, r = cv2.split(img_bgr)
    b.fill(255)
    img_bgr = cv2.merge((b,g,r))
    img_blur = cv2.medianBlur(img_bgr,5)

    return img_blur

def preprocessing_drum(img_bgr):
    print_result('preprocessing_gate')
    b, g, r = cv2.split(img_bgr)
    r.fill(255)
    img_bgr = cv2.merge((b,g,r))
    img_blur = cv2.medianBlur(img_bgr,5)

    return img_blur
def range_str2list(str):
    str = str.split(',')
    return np.array([int(str[0]), int(str[1]), int(str[2])], np.uint8)


def print_result(msg):
    print('<------------- ') + str(msg) + (' ------------->')


def delete_color(img_hsv, color, camera):
    lower, upper = getColor(color, camera)
    print(lower, upper)
    if not lower is None:
        resHSV = cv2.inRange(img_hsv, [0, 0, 0], [180, 255, 255])
    else:
        h, s, v = cv2.split(img_hsv)
        resHSV = cv2.inRange(img_hsv, lower, upper)
    return resHSV


def get_color(color,time, mission):
    lower = None
    upper = None
    color_list = ['orange', 'white', 'yellow', 'red', 'black', 'green','navy']

    for c in color_list:
        if color == c:
            lower = rospy.get_param(
                '/color_range_' + str(mission) + '/color_' + str(time) + '/lower_' + c)
            upper = rospy.get_param(
                '/color_range_' + str(mission) + '/color_' + str(time) + '/upper_' + c)
            break
    if not lower is None:
        lower = range_str2list(lower)
        upper = range_str2list(upper)
    print(lower,upper)
    return lower, upper


def find_shape(cnt, req):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
    if len(approx) == 3 and req == 'tri':
        return True
    elif len(approx) == 4 and req == 'rect':
        return True
    elif len(approx) >= 10 and req == 'cir':
        return True
    elif len(approx) == req:
        return True
    return False


def cut_contours(M, w, h, range_w, range_h):
    cx = None
    cy = None
    try:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
    except:
        print 'err'
    if cx is None:
        return True
    if cx <= range_w or cy <= range_h or cx >= w - range_w or cy >= h - range_h:
        return True
    return False


def brightness(img_bgr, brightnessValue):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = np.int16(v)
    v = np.clip(v + brightnessValue, 0, 255)
    v = np.uint8(v)
    hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def clip_v(img_bgr, min, max):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = np.clip(v, min, max)
    hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def equalization_bgr(img_bgr):
    b, g, r = cv2.split(img_bgr)
    b = cv2.equalizeHist(b)
    g = cv2.equalizeHist(g)
    r = cv2.equalizeHist(r)
    equ_bgr = cv2.merge((b, g, r))
    return equ_bgr


def equalization_hsv(img_hsv):
    h, s, v = cv2.split(img_hsv)
    s = cv2.equalizeHist(s)
    v = cv2.equalizeHist(v)
    equHSV = cv2.merge((h, s, v))
    return equHSV


def equalization_gray(img_gray):
    equ_gray = cv2.equalizeHist(img_gray)

    return equ_gray


def clahe(img_bgr):
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Lab)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))
    res_bgr = cv2.cvtColor(lab, cv2.COLOR_Lab2BGR)
    return res_bgr


def Oreintation(moment):
    tmp = pow((moment['mu20'] - moment['mu02']), 2)
    tmp = math.sqrt(tmp + (4 * pow(moment['mu11'], 2)))

    k = moment['mu02'] - moment['mu20']

    l = 2 * moment['mu11']

    rad_maj = math.atan2(k + tmp, l)
    rad_min = math.atan2(k - tmp, l)
    return rad_maj, rad_min


def nothing(variable):
    pass



def get_mode(data):
    data = np.array(data)
    if len(data.shape) > 1:
        data = data.ravel()
    try:
        mode = statistics.mode(data)
    except ValueError:
        mode = None
    return mode


def get_cv(data):
    mean = cv2.mean(data)[0]
    sd = cv2.meanStdDev(data, mean)[0]
    return sd / mean



def publish_result(img, type, topicName):
    if img is None:
        img = np.zeros((200, 200))
        type = "gray"
    bridge = CvBridge()
    pub = rospy.Publisher(
        str(topicName), Image, queue_size=10)
    if type == 'gray':
        msg = bridge.cv2_to_imgmsg(img, "mono8")
    elif type == 'bgr':
        msg = bridge.cv2_to_imgmsg(img, "bgr8")
    pub.publish(msg)


def adjust_gamma(img_bgr=None, gamma=1):
    if img_bgr is None:
        print('given value to img_bgr argument\n' +
              'adjust_gamma_by_value(img_bgr, gamma)')

    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) *
                      255 for i in np.arange(0, 256)]).astype("uint8")

    return cv2.LUT(img_bgr, table)


def adjust_gamma_by_v(img_bgr=None):
    if img_bgr is None:
        print('given value to img_bgr argument\n' +
              'adjust_gamma_by_value(img_bgr)')
        return

    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    vMean = cv2.mean(v)[0]
    gamma = vMean / 13
    # print 'gamma : ' + str(gamma)

    if gamma == 0:
        g = 1.0
    else:
        g = gamma / 10.0
    invGamma = 1.0 / g
    table = []
    for i in np.arange(0, 256):
        table.append(((i / 255.0) ** invGamma) * 255)
    table = np.array(table).astype('uint8')
    res = cv2.LUT(img_bgr, table)
    return res


def get_kernel(shape='rect', ksize=(5, 5)):
    if shape == 'rect':
        return cv2.getStructuringElement(cv2.MORPH_RECT, ksize)
    elif shape == 'ellipse':
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ksize)
    elif shape == 'plus':
        return cv2.getStructuringElement(cv2.MORPH_CROSS, ksize)
    else:
        return None


def erode(imgBin, ker):
    return cv2.erode(imgBin, ker, iterations=1)


def dilate(imgBin, ker):
    return cv2.dilate(imgBin, ker, iterations=1)


def close(imgBin, ker):
    return cv2.morphologyEx(imgBin, cv2.MORPH_CLOSE, ker)


def open_morph(imgBin, ker):
    return cv2.morphologyEx(imgBin, cv2.MORPH_OPEN, ker)


def callback_raw(ros_data):
    global image
    width = 640
    height = 512
    bridge = CvBridge()
    image = cv2.resize(bridge.imgmsg_to_cv2(ros_data, "bgr8"), (width, height))


def callback_compressed(ros_data):
    global image
    width = 640
    height = 512
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image = cv2.resize(cv2.imdecode(np_arr, 1), (width, height))
