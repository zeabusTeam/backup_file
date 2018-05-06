#!/usr/bin/python2.7

import rospy, os
import math
from std_msgs.msg import Bool
from aicontrol import AIControl
from navigate import Gate
from flare import Flare
from drum import Drum
import constants_fix as cons


if __name__=='__main__':
    rospy.init_node('main_ai')
    print 'init node completed'
    count = 0

    aicontrol = AIControl()
    start_yaw = aicontrol.auv_state[5]
    drum_x = cons.DRUM_X*math.cos(aicontrol.auv_state[5])+cons.DRUM_Y*math.cos(aicontrol.auv_state[5]+math.radians(90))
    drum_y = cons.DRUM_X*math.sin(aicontrol.auv_state[5])+cons.DRUM_Y*math.sin(aicontrol.auv_state[5]+math.radians(90))

    flare_x = cons.FLARE_X*math.cos(aicontrol.auv_state[5])+cons.FLARE_Y*math.cos(aicontrol.auv_state[5]+math.radians(90))
    flare_y = cons.FLARE_X*math.sin(aicontrol.auv_state[5])+cons.FLARE_Y*math.sin(aicontrol.auv_state[5]+math.radians(90))
    print 'gate'
    gate_mission = Gate()
    aicontrol.depthAbs(-1.2)
    #GATE
    aicontrol.driveX(4)
    gate_mission.run()


    #SET XY BEFORE DO DRUM
    aicontrol.fixXY(drum_x, drum_y)
    drum_mission.run()
    aicontrol.turnRelative(cons.DRUM_DEGREE)
    #DRUM

    #SET XY BEFORE DO FLARE
    aicontrol.fixXY(flare_x, flare_y)
    #FLARE

    #GET TO SURFACE
    print 'MISSIONS COMPLETED'
    aicontrol.depthAbs(-0.1)
