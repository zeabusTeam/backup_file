#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage

img = None


def callback(ros_data):
    global img
    width = 640
    height = 320
    np_arr = np.fromstring(ros_data.data, np.uint8)
    img = cv2.resize(cv2.imdecode(
        np_arr, 1), (width, height))


def harris_corner_detector():
    global img

    while not rospy.is_shutdown():
        if img is None:
            print('image is none')
            continue
        else:
            break
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, 2, 3, 0.001)
    # result is dilated for marking the corners, not important
    dst = cv2.dilate(dst, None)
    # Threshold for an optimal value, it may vary depending on the image.
    print img
    img[dst > 0.01 * dst.max()] = [0, 0, 255]
    print img
    cv2.imshow('dst', dst)
    cv2.imshow('img', img)

    k = cv2.waitKey(0) & 0xff
    #     if k == ord('q'):
    #         break
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('harris_corner')
    sub = rospy.Subscriber('/stereo/left/image_raw/compressed',
                           CompressedImage, callback, queue_size=10)
    harris_corner_detector()
