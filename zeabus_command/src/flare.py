#!/usr/bin/python2

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_example.msg import vision_flare
from zeabus_example.srv import vision_srv_flare
from aicontrol import AIControl
import constants as cons

class Flare(object):
    def __init__(self):
        self.data = vision_flare()

        print '<===INIT FLARE===>'
        #rospy.init_node('flare_node')
        print 'INITING AI'
        self.aicontrol = AIControl()
        print 'FLARE AIControl init completed'
        self.detect_flare = rospy.ServiceProxy('vision_flare', vision_srv_flare)

    def getData(self):
        self.data = self.detect_flare(String('flare'), String('flare'))
        self.data = self.data.data

    def run(self):
        print '<===DOING FLARE===>'
        auv = self.aicontrol
        print '<----------drive z---------->'
        auv.depthAbs(cons.FLARE_DEPTH, 0.1)
        print '<----------drive completed---------->'

        mode = 0
        count = 0
        reset = 0
        center = 0
        count_pos_r = 0
        count_pos_l = 0
        flare_pos = cons.FLARE_POS

        while not rospy.is_shutdown() and not mode == -1:
            self.getData()
            appear = self.data.appear
            cx = self.data.cx+cons.VISION_FRONT_FIX
            area = self.data.area
            if mode == 0:
                print '<---GET IN TO THE FLARE AREA--->'
                print 'MODE 1'
                '''
                auv.driveX(cons.FLARE_X)
                auv.driveY(cons.FLARE_Y)
                auv.turnRelative(cons.FLARE_DEGREE)
                '''
                start_point = auv.auv_state[:1]
                mode = 1

            if mode == 1:
                print '---mode 1---'
                current_point = auv.auv_state[0:1]
                if ((start_point[0]-current_point[0])**2+(start_point[1]-current_point[1]))**0.5 > 5:
                    print 'ABORT MISSION'
                    mode = -1
                if appear:
                    count += 1
                    reset = 0
                    print 'FOUND FLARE: %d'%(count)
                    if cx < 0:
                        count_pos_l += 1
                        print 'CPosL: %d'%(count_pos_l)
                    elif cx > 0:
                        count_pos_r += 1
                        print 'CPosR: %d'%(count_pos_r)
                    if CPosL >= 5:
                        flare_pos = 'left'
                        count_pos_l = 0
                        count_pos_r = 0
                    elif CPosR >= 5:
                        flare_pos = 'right'
                        count_pos_l = 0
                        count_pos_r = 0

                else:
                    reset += 1
                    print 'FLARE NOT FOUND: %d'%(reset)

                if count >= 3:
                    print 'MODE 2'
                    mode = 2
                if reset >= 3:
                    count = 0
                    reset = 0
                auv.move(flare_pos, cons.AUV_SPEED_FIND_FLARE)

            if mode == 2:
                print '----mode 2----'
                print 'cx: %f'%(cx)
                print 'area: %f'%(area)
                if cx < 0:
                    auv.move('left', cons.AUV_SPEED_FIND_FLARE*abs(cx))
                    count_pos_l += 1
                elif cx < 0:
                    auv.move('right', cons.AUV_SPEED_FIND_FLARE*abs(cx))
                    count_pos_r += 1

                if count_pos_l >= 5:
                    flare_pos = 'left'
                    count_pos_l = 0
                    count_pos_r = 0
                elif count_pos_r >= 5:
                    flare_pos = 'right'
                    count_pos_l = 0
                    count_pos_r = 0

                if -cons.VISION_ERROR-cons.VISION_ERROR_FLARE <= cx <= cons.VISION_ERROR+cons.VISION_ERROR_FLARE:
                    print '<<CENTER>>'
                    center += 1
                    auv.stop()
                else:
                    center = 0
                if center >= 3:
                    mode = 3
                    print 'MODE 3'
            if mode == 3:
                print '----mode 3----'
                print 'area: %f'%(area)
                if -cons.VISION_ERROR-cons.VISION_ERROR_FLARE > cx > cons.VISION_ERROR+cons.VISION_ERROR_FLARE:
                    reset += 1
                    print 'NOT CENTER'
                elif -cons.VISION_ERROR-cons.VISION_ERROR_FLARE <= cx <= cons.VISION_ERROR+cons.VISION_ERROR_FLARE:
                    print '<<<CENTER>>>'
                    reset = 0
                if reset >= 3:
                    print 'MODE 2'
                    mode = 2
                if area < 0.005 and area != 0:
                    print 'TOO FAR'
                    auv.driveX(1)
                elif area >= 0.01:
                    mode = 4
                    print 'MODE 4'
            if mode == 4:
                print '---mode 4---'
                if count > 0:
                    auv.move('forward', cons.AUV_SPEED)
                    if not appear:
                        count -= 1
                        print '<<<NOT FOUND FLARE>>>'
                        driveX(2)
                    else:
                        count = 5
                        print 'FOUND FLARE AGAIN'
                else:
                    mode = -1
                    print 'END MISSION'
        auv.stop()

if __name__=='__main__':
    mission_flare = Flare()
    mission_flare.run()
    print 'finish flare'
