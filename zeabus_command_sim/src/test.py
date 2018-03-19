#!/usr/bin/python2

from aicontrol import AIControl
import rospy 

global aicontrol


if __name__=='__main__':
    aicontrol = AIControl()
    cmd = raw_input('command \'stop or 1\' , \'depth or 2\' , \'move or 3\' : ,\'turn or 4\'')
    if cmd == 'stop' or cmd == '1': 
        aicontrol.stop()
    elif cmd == 'depth' or cmd == '2': 
        pos = int(input('position : '))
        aicontrol.depthAbs(pos)
    elif cmd == 'move' or cmd == '3': 
        pos = raw_input('pos : ')
        speed = int(input('speed : '))        
        count = int(input('count : '))
        while not rospy.is_shutdown() and not aicontrol.isFail(count):
            aicontrol.move(pos,speed)
            # rospy.loginfo('x: %f\ny: %f\nz: %f'%(aicontrol.auv_state[0], aicontrol.auv_state[1], aicontrol.auv_state[2]))
            count-=1
        aicontrol.stop()
    elif cmd == 'turn' or cmd == '4':
	    angle = int(input('angle : '))  
	    aicontrol.turnRelative(angle)
    rospy.sleep(1)
        
