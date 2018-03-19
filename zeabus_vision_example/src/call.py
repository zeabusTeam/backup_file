#!/usr/bin/env python
import cv2
import rospy
from zeabus_example.msg import *
from zeabus_example.srv import *
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('call_service')
    serviceName = 'vision_gate'
    print('wait service')
    rospy.wait_for_service(serviceName)
    print('service start')

    call = rospy.ServiceProxy(serviceName, vision_srv_gate)
    while not rospy.is_shutdown():
        res = call(String('gate'), String('gate'))
        print(res)
        rospy.sleep(0.1)
