#!/usr/bin/python2

import rospy, roslaunch, os
from std_msgs.msg import Bool
from zeabus_controller.srv import *


kill = 0
on = 0
srv = rospy.ServiceProxy('mode_control', control_mode)

def listener():
    rospy.init_node('switch_node')
    rospy.Subscriber('/planner_switch', Bool, isOn)
    print 'Switch.py is running'
    rospy.spin()

def isOn(is_on):
    global kill, on, srv
    if kill >= 30 and not is_on.data:
        srv(Bool(False))
        ###
        os.system('rosnode kill zeabus_command')
        #print 'KILL'
    elif on >= 30 and is_on.data:
        srv(Bool(True))
        os.system('roslaunch zeabus_command ai.launch')
        #print 'START CONTROL'

    elif not is_on.data:
        kill += 1
        print 'kill: %d'%(kill)
        on = 0
    else:
        kill = 0
        on += 1
        print 'on: %d'%(on)


if __name__=='__main__':
    listener()
