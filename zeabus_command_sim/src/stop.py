#!/usr/bin/python2

from aicontrol import AIControl as aicontrol
import rospy

global AIControl

if __name__=='__main__':
    aicontrol = AIControl() 
    count=5
    while not rospy.is_shutdown() and not self.aicontrol.isFail(count):
        aicontrol.stop()
        count-=1
    rospy.sleep(5)
        