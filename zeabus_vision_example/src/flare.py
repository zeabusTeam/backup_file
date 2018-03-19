#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import math
import sys
from operator import itemgetter
from vision_lib import *
from sensor_msgs.msg import CompressedImage
from zeabus_example.msg import *
from zeabus_example.srv import *
from std_msgs.msg import String
img = None
sub_sampling = 1
img_res = None

def mission_callback(msg):
    print_result('mission_callback')

    req = msg.req.data
    task = msg.task.data

    print('task:', str(task), 'request:', str(req))
    if task == 'flare':
        return find_flare()


def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (0, 0),
                     fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()


def message(x=0, area=0, appear=False):
    global c_img
    print(x,area,appear)
    m = vision_flare()
    m.cx = x
    m.area = area
    m.appear = appear
    print(m)
    return m


def get_object(img_bgr):
    lower, upper = get_color('yellow', 'morning', 'flare')
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    print(lower, upper)
    obj = cv2.inRange(hsv, lower, upper)
    return obj


def get_bg(img_bgr):
    lower, upper = get_color('black', 'morning', 'flare')
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    print(lower, upper)
    bg = cv2.inRange(hsv, lower, upper)
    return bg


def get_centroid(img_bin,img_pre):
    global img, img_res, hsv, c_img, msg
    print_result('get_centroid')

    area_min = 300
    w_h_min_ratio = 5

    _, th = cv2.threshold(img_bin, 127, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(
        th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    result_list = []

    font = cv2.FONT_HERSHEY_SIMPLEX
    w_h_ratio = 0
    for c in contours:
        area = cv2.contourArea(c)
        xx,yy,ww,hh = cv2.boundingRect(c)
        rect = (x, y), (w, h), angle = cv2.minAreaRect(c)
        moment = cv2.moments(c)
        _,angle = Oreintation(moment)
        cv2.putText(img_res, str(int(area)) + ' ' + str(int(w)) + ' ' + str(int(h)) + ' ' + str(int(angle)),
                    (int(x), int(y)), font, 1.5, (0, 255, 255), 1, cv2.LINE_AA)
        # cv2.drawContours(img_res, c, -1, (0, 0, 255), 2)
        if w > 0 and h > 0:
            w_h_ratio = max(w / h, h / w)

        if area <= area_min:
            continue
        if w_h_ratio <= w_h_min_ratio:
            continue
        # if not(-89 <= angle <= -70 or -2 <= angle <= 5):
        if angle > 5 or angle < -5 :
            continue
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img_res, [box], 0, (255, 255, 0), 3)
        result_list.append([x, y, area,xx,yy,ww,hh])

    r_img, c_img = img_bin.shape

    x_result = 0
    area_result = 0
    y_result = int(r_img/2)

    if len(result_list) >= 2:
        diiff_y_min = 1000000
        result_list = sorted(result_list, key=itemgetter(1))

        for i in range(1, len(result_list)):
            x_j, y_j, area_j,_,_,_,_ = result_list[i-1]
            x_i, y_i, area_i,_,_,_,_ = result_list[i]
            diiff_y = abs(y_i - y_j)
            if x_i == x_j and y_i == y_j:
                continue
            if diiff_y < diiff_y_min:
                x_result = int((x_i + x_j) / 2)
                area_result = ((area_i + area_j) / 2.0)
        cv2.circle(img_res, (x_result, y_result), 10, (255, 0, 255), -1)
        x_result = (x_result - c_img/2.0)/(c_img/2.0)
        area_result = area_result/(r_img*c_img*1.0)
        print(x_result,area_result)
        return message(x_result, area_result, True)
    
    elif len(result_list) == 1:
        x, y, _ ,xx,yy,ww,hh= result_list[0]
        bgr = img_pre

        x = int(x)
        y = int(y)
        cv2.circle(img_res, (int(x), int(y)), 10, (0, 255, 0), -1)
        x_result = (x - c_img/2.0)/(c_img/2.0)
        area_result = area_result/(r_img*c_img*1.0)
        print(x_result,area_result)
        return message(x_result, area_result, True)
    else:
        print('0')
        return message()


def remove_noise(img_bin):
    kernel = get_kernel('rect', (5, 9))
    erode = cv2.erode(img_bin, kernel)

    kernel = get_kernel('rect', (5, 15))
    dilate = cv2.dilate(erode, kernel)

    kernel = get_kernel('rect', (1, 10))
    erode = cv2.erode(dilate, kernel)

    res_bin = erode
    return res_bin


def get_frame(mask):
    global img_res
    r, c = mask.shape
    frame = np.zeros((r, c), dtype=np.uint8)
    frame = cv2.rectangle(frame, (int(c * 0.05), int(r * 0.15)),
                          (c - int(c * 0.05), r - int(r * 0.05)), (255, 255, 255), -1)
    img_res = cv2.rectangle(img_res, (int(c * 0.05), int(r * 0.15)),
                            (c - int(c * 0.05), r - int(r * 0.1)), (255, 0, 255), 2)
    res = mask & frame
    return res


def find_flare():
    global img, img_res, hsv, msg

    while img is None:
        print('img is none.\nPlease check topic name or check camera is running')
    print(img.shape)

    img_pre = preprocessing_flare(img)
    print_result('finish preprocessing_flare')

    obj = get_object(img_pre)
    bg = get_bg(img_pre)

    mask = obj
    mask_clear = remove_noise(mask)
    mask_with_frame = get_frame(mask_clear)
    
    msg = get_centroid(mask_with_frame,img_pre)

    print_result('before publisher')
    # cv2.imshow('mask',mask)
    publish_result(mask, 'gray', 'mask')
    # cv2.imshow('mask_clear',mask_clear)
    publish_result(mask_clear, 'gray', 'mask_clear')
    # cv2.imshow('mask_with_frame',mask_with_frame)
    publish_result(mask_with_frame, 'gray', 'mask_with_frame')
    # cv2.imshow('img',img)
    publish_result(img, 'bgr', 'img')
    # cv2.imshow('img_pre',img_pre)
    publish_result(img_pre, 'bgr', 'img_pre')
    # cv2.imshow('img_result',img_result)
    publish_result(img_res, 'bgr', 'img_result')

    return msg


def main():
    rospy.init_node('vision_flare', anonymous=True)
    #image_topic = "/top/center/image_raw/compressed"
    image_topic = "/top/center/image_raw/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    rospy.Service('vision_flare', vision_srv_flare(), mission_callback)
    rospy.spin()


if __name__ == '__main__':
    main()