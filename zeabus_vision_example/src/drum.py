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
    global req
    print_result('mission_callback')

    req = msg.req.data

    task = msg.task.data

    print('task:', str(task), 'request:', str(req))
    if task == 'drum' and req == 'drum':
        return find_drum()
    if task == 'drum' and req == 'mat':
        return find_mat()

def get_centroid_drum(img_bin):
    global img, img_res
    print_result('get_centroid')

    area_min = 1000
    area_max = 4000000
    radius_min = 1
    area_ratio = 0.80

    _, th = cv2.threshold(img_bin, 127, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(
        th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0 :
        return message()
    
    else :
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.drawContours(img_res, contours, -1, (0, 0, 255), 2)
        result_list = []

        for c in contours:
            area_cnt = cv2.contourArea(c)
            (x, y), r = cv2.minEnclosingCircle(c)
            area_cir = math.pi * r * r

            if area_cnt <= area_min or area_cnt >= area_max:
                continue

            if area_cnt / area_cir <= area_ratio:
                continue

            x = int(x)
            y = int(y)
            r = int(r)

            cv2.circle(img_res, (x, y), r, (0, 255, 0), 1)
            cv2.putText(img_res, str(int(area_cnt)) + ' ' + str(int(r)),
                        (int(x), int(y)), font, 3, (0, 0, 255), 2, cv2.LINE_AA)
            result_list.append([x, y, r, area_cnt])

        r_img, c_img = img_bin.shape

        area_result = 0
        x_result = 0
        y_result = 0
        r_result = 0

        if len(result_list) <= 0:
            print('0')
            return message()
        else:
            result_list = sorted(result_list, key=itemgetter(3), reverse=True)
            x, y, r, area = result_list[0]
            cv2.circle(img_res, (x, y), r, (255,100, 0), 10)
            area_result = area/(c_img * r_img * 1.0)
            x_result = (x - c_img/2.0)/(c_img/2.0)
            y_result = (y - r_img/2.0)/(r_img/2.0)
            return message(x_result, y_result, area_result, True)
            print(result_list[0])

def get_centroid(img_bin):
    global img, img_res, req
    print_result('get_centroid')
    
    area_min = 600000 
    
    _, th = cv2.threshold(img_bin, 127, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(
        th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0 :
        return message()
    else :
        result_list = []
        font = cv2.FONT_HERSHEY_SIMPLEX
        for c in contours:
            area = cv2.contourArea(c)
            xx,yy,ww,hh = cv2.boundingRect(c)
            rect = (x, y), (w, h), angle = cv2.minAreaRect(c)
            if area <= area_min:
                continue
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img_res,[box], 0, (0,255,0), 3)
            if w > 0 and h > 0:
                w_h_ratio = max(w / h, h / w)
            if w_h_ratio <= 1 :
                continue
            if area <= area_min:
                continue
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img_res, [box], 0, (255,255,0), 3)
            result_list.append([x, y, area,xx,yy,ww,hh])
        r_img, c_img = img_bin.shape
        x_result = 0
        y_result = 0
        area_result = 0
        if len(result_list) >= 2:
            result_list = sorted(result_list, key=itemgetter(2), reverse=True)
        elif len(result_list) != 0 : 
            x, y, area, xx, yy, ww, hh = result_list[0]
            cv2.putText(img_res, str(int(area)) + ' ' + str(int(ww)) + ' ' + str(int(hh)) + ' ' + str(int(angle)),
            (int(x), int(y)), font, 1.5, (0, 255, 255), 1, cv2.LINE_AA)
        x = int(x)
        y = int(y)
        cv2.circle(img_res, (x, y), 10, (255, 0, 255), -1)
        x_result = (x - c_img/2.0)/(c_img/2.0)
        y_result = (y - r_img/2.0)/(r_img/2.0)
        area_result = area/(r_img * c_img * 1.0)
        if area_result < 0.04 :
            x_result = 0
            y_result = 0
            area_result = 0
            return message(x_result,y_result,area_result,False)
        return message(x_result,y_result,area_result,True)

# def get_centroid_drum(img_bin):
    # global img, img_res
    # print_result('get_centroid')
# 
    # area_min = 5000
    # 
    # _, th = cv2.threshold(img_bin, 127, 255, cv2.THRESH_BINARY)
    # _, contours, _ = cv2.findContours(
        # th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # print('contours',contours)
    # 
    # result_list = []
# 
    # font = cv2.FONT_HERSHEY_SIMPLEX
    # for c in contours:
        # print('in loop')
        # area = cv2.contourArea(c)
        # xx,yy,ww,hh = cv2.boundingRect(c)
        # rect = (x, y), (w, h), angle = cv2.minAreaRect(c)
        # 
        # if area <= area_min:
            # continue
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # cv2.drawContours(img_res,[box], 0, (0,255,0), 3)
# 
        # if w > 0 and h > 0:
            # w_h_ratio = max(w / h, h / w)
# 
        # if area <= area_min:
            # continue
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # cv2.drawContours(img_res, [box], 0, (255,255,0), 3)
        # result_list.append([x, y, area,xx,yy,ww,hh])
# 
# 
    # r_img, c_img = img_bin.shape
    # x_result = 0
    # y_result = 0
    # area_result = 0
# 
    # x, y, area, xx, yy, ww, hh = result_list[0]
# 
    # x = int(x)
    # y = int(y)
# 
    # x_result = (x - c_img/2.0)/(c_img/2.0)
    # y_result = (y - r_img/2.0)/(r_img/2.0)
    # area_result = area/(r_img * c_img * 1.0)
    # return message(x_result,y_result,area_result,True)

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (0, 0),
                     fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()


def get_object_drum_blue(img_bgr):
    lower, upper = get_color('orange', 'morning', 'drum')
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    obj = cv2.inRange(hsv, lower, upper)
    return obj

def get_object_mat(img_bgr):
    lower, upper = get_color('green', 'morning', 'drum')
    print('mat',lower,upper)
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    obj = cv2.inRange(hsv, lower, upper)
    return obj

def get_object_drum_red(img_bgr):
    lower, upper = get_color('red', 'morning', 'drum')
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    obj = cv2.inRange(hsv, lower, upper)
    return obj


def get_bg(img_bgr):
    lower, upper = get_color('black', 'morning', 'drum')
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    bg = cv2.inRange(hsv, lower, upper)
    return bg


def remove_noise(img_bin):
    kernel = get_kernel('rect',(5,5))
    dilate = cv2.dilate(img_bin, kernel,iterations=2)

    res_bin = dilate
    # for new bag because current bag is easy.

    return res_bin
    # return dilate


def find_drum():
    global img, img_res
    while img is None:
        print('img is none :: check topic')
    print(img.shape)

    img_pre = preprocessing_drum(img)
    print_result('finish preprocessing')
    print(img_pre)

    obj_red = get_object_drum_red(img_pre)
    print('obj_red',obj_red)
    obj_blue = get_object_drum_blue(img_pre)
    print('obj_blue',obj_blue)
    obj_green = get_object_mat(img_pre)
    bg = get_bg(img_pre)

    mask = obj_blue
    print('mask',mask)
    # mask_clear = remove_noise(mask)
    # print('mask_clear',mask_clear)

    msg = get_centroid_drum(mask)
    # get_centroid(mask_clear)
    print(msg)

    print_result('before publisher')
    publish_result(bg, 'gray', 'bg')
    publish_result(mask, 'gray', 'mask')
    publish_result(obj_blue, 'gray', 'blue')
    publish_result(obj_red, 'gray', 'red')
    publish_result(obj_green, 'gray', 'green')
    # publish_result(mask_clear, 'gray' , 'mask_clear')
    publish_result(img, 'bgr', 'img')
    publish_result(img_pre, 'bgr', 'img_pre')
    publish_result(img_res, 'bgr', 'img_res')

    return msg

def find_mat():
    global img, img_res
    while img is None:
        print('img is none :: check topic')
    print(img.shape)

    img_pre = preprocessing_drum(img)
    print_result('finish preprocessing')
    print(img_pre)

    obj_red = get_object_drum_red(img_pre)
    obj_blue = get_object_drum_blue(img_pre)
    obj_green = get_object_mat(img_pre)
    bg = get_bg(img_pre)

    mask = obj_green - obj_blue + obj_red - bg
    mask_clear = remove_noise(mask)
    print('mask_clear',mask_clear)

    msg = get_centroid(mask_clear)
    # get_centroid_mat(mask_clear)
    print(msg)

    print_result('before publisher')
    publish_result(mask, 'gray', 'mask')
    publish_result(bg, 'gray', 'bg')
    publish_result(obj_blue, 'gray', 'blue')
    publish_result(obj_red, 'gray', 'red')
    publish_result(obj_green, 'gray', 'green')
    publish_result(mask_clear, 'gray' , 'mask_clear')
    publish_result(img, 'bgr', 'img')
    publish_result(img_pre, 'bgr', 'img_pre')
    publish_result(img_res, 'bgr', 'img_res')

    return msg

def message(x=0, y=0, area=0, appear=False):
    global c_img, r_img
    print(x, y, area, appear)
    m = vision_drum()
    m.cx = x
    m.cy = y
    m.area = area
    m.appear = appear
    print(m)
    return m


def main():
    rospy.init_node('vision_drum', anonymous=True)
    image_topic = "/bottom/left/image_raw/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    rospy.Service('vision_drum', vision_srv_drum(), mission_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
