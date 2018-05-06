#!/usr/bin/env python
import cv2
import rosbag
import rospy
import numpy as np
import math
from sensor_msgs.msg import CompressedImage


class bag2img:

    def __init__(self, filename):
        rospy.init_node('bag2img')
        self.topic_l = '/leftcam_top/image_raw/compressed'
        self.topic_r = '/rightcam_top/image_raw/compressed'
        self.bag = rosbag.Bag(filename)

        self.count = 0

    def left_callback(self, msg):
        arr = np.fromstring(msg.data, np.uint8)
        self.img_l = cv2.imdecode(arr, 1)

    def right_callback(self, msg):
        arr = np.fromstring(msg.data, np.uint8)
        self.img_r = cv2.imdecode(arr, 1)

    def capture(self):
        while not rospy.is_shutdown():
            key = cv2.waitKey(1) & 0xff
            name_l = 'left-' + str(self.count) + '.jpg'
            name_r = 'right-' + str(self.count) + '.jpg'
            cv2.imwrite(name_l, self.img_l)
            cv2.imwrite(name_r, self.img_r)
            self.count += 1
            if key == ord('q'):
                break
            rospy.sleep(1)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    bag2jpg = bag2img('calibrate.bag')
    bag2jpg.capture()
