#!/usr/bin/env python
import cv2
import rospy
from zeabus_vision_srv_msg.msg import *
from zeabus_vision_srv_msg.srv import *
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('call_service')
    serviceName = 'vision_bouy'
    # serviceName = 'vision_navigate'
    # serviceName = 'vision_squid'
    # binsrv = 'vision_bin'
    print('wait service')
    rospy.wait_for_service(serviceName)
    print('service start')

    # call = rospy.ServiceProxy(serviceName, vision_srv_navigate)
    call = rospy.ServiceProxy(serviceName, vision_srv_bouy)
    # call = rospy.ServiceProxy(serviceName, vision_srv_default)
    while not rospy.is_shutdown():
        # res = call(String('Navigate'), String('bot'))

        # res = call(String('squid'), String('b'))
        # res = call(String('bouy'), String('a'))
        res = call(String('bouy'), String('o'))
        # res = call(String('bouy'), String('g'))
        # res = res.data
        # print res.x
        # print res.y
        # print res.appear
        print res

        rospy.sleep(0.1)
