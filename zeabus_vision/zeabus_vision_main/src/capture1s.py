#!/usr/bin/env python
import cv2
import rosbag
import rospy
import numpy as np
import math
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError


class capture1s:

    def __init__(self):
        rospy.init_node('capture1s')
        self.subTopicL = '/stereo/left/image_raw'
        self.subTopicR = '/stereo/right/image_raw'
        self.count = 0
        self.imgL = None
        self.imgR = None
        self.path = '/home/zeabus/image_calibration/'
        self.bridge = CvBridge()

        self.subLeft = rospy.Subscriber(
            self.subTopicL, Image, self.left_callback)
        self.subRight = rospy.Subscriber(
            self.subTopicR, Image, self.right_callback)

    def left_callback(self, data):
        # print(data)
        self.imgL = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # self.imgL = cv2.imdecode(cv_image, 1)
        # print self.imgL

    def right_callback(self, data):
        # print(data)
        self.imgR = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # self.imgR = cv2.imdecode(cv_image, 1)
        # print self.imgR

    def capture(self):
        while self.imgL is None or self.imgR is None:
            print('frame is none')
            # rospy.sleep(0.01)

        while not rospy.is_shutdown():
            if self.imgL is None or self.imgR is None:
                continue
            key = cv2.waitKey(1) & 0xff
            if key == ord('c'):
                nameL = self.path + 'left/left-' + str(self.count) + '.png'
                nameR = self.path + 'right/right-' + str(self.count) + '.png'
                cv2.imwrite(nameL, self.imgL)
                cv2.imwrite(nameR, self.imgR)
                self.count += 1
            cv2.imshow('left', self.imgL)
            cv2.imshow('right', self.imgR)

            if key == ord('q'):
                break
            print(self.count)
            # rospy.sleep(0.1)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    bag2jpg = capture1s()
    bag2jpg.capture()
