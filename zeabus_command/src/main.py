#!/usr/bin/python2.7

import rospy
import math
from std_msgs.msg import Bool
from aicontrol import AIControl
from navigate import Gate
from flare import Flare
from drum import Drum
import constants as cons

if __name__=='__main__':
    rospy.init_node('main_ai')
    print 'init node completed'
    count = 0

    rospy.wait_for_service('vision_gate')
    rospy.wait_for_service('vision_drum')
    rospy.wait_for_service('vision_flare')
    aicontrol = AIControl()
    start_yaw = aicontrol.auv_state[5]
    gate_mission = Gate()
    print 'gate'
    drum_mission = Drum()
    print 'drum'
    flare_mission = Flare()
    print 'flare'
    drum_x = cons.DRUM_X*math.cos(aicontrol.auv_state[5])+cons.DRUM_Y*math.cos(aicontrol.auv_state[5]+math.radians(90))
    drum_y = cons.DRUM_X*math.sin(aicontrol.auv_state[5])+cons.DRUM_Y*math.sin(aicontrol.auv_state[5]+math.radians(90))

    flare_x = cons.FLARE_X*math.cos(aicontrol.auv_state[5])+cons.FLARE_Y*math.cos(aicontrol.auv_state[5]+math.radians(90))
    flare_y = cons.FLARE_X*math.sin(aicontrol.auv_state[5])+cons.FLARE_Y*math.sin(aicontrol.auv_state[5]+math.radians(90))
    #GATE
    gate_mission.run()

    #SET XY BEFORE DO FLARE
    aicontrol.fixXY(flare_x, flare_y)
    #aicontrol.turnRelative(cons.FLARE_DEGREE)
    #FLARE
    flare_mission.run()

    #SET XY BEFORE DO DRUM
    aicontrol.fixXY(drum_x, drum_y)
    #aicontrol.turnRelative(cons.DRUM_DEGREE)
    #DRUM
    drum_mission.run()

    #GET TO SURFACE
    print 'MISSIONS COMPLETED'
    aicontrol.depthAbs(-0.1)
