#!/usr/bin/python2.7

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_example.srv import vision_srv_drum
from zeabus_example.msg import vision_drum
from aicontrol import AIControl
import constants as cons

class Drum(object):
    def __init__(self):
        self.data = vision_drum()

        print '<===INIT DRUM===>'
        #rospy.init_node('drum_node')
        self.aicontrol = AIControl()
        rospy.wait_for_service('vision_drum')
        self.detect_drum = rospy.ServiceProxy('vision_drum', vision_srv_drum)

    def detect(self, req):
        self.data = self.detect_drum(String('drum'), String(req))
        self.data = self.data.data

    def run(self):
        print '<===DOING DRUM===>'
        auv = self.aicontrol
        # - drive depth
        auv.depthAbs(cons.DRUM_DEPTH)
        print '<---drive complete--->'
        # -------------

        count = 0
        reset = 0
        mode = 0
        switch_mode = 0
        drum_pos = cons.DRUM_POS

        while not rospy.is_shutdown() and not mode == -1:
            if mode == 0:
                print '<---get to detect range--->'

                print '<---stop in detect range--->'
                mode = 1
                print 'MODE 1'

            if mode == 1:
                self.detect('mat')
                appear = self.data.appear
                print '---mode 1---'
                if appear:
                    count += 1
                    reset = 0
                    print 'FOUND MAT: %d'%(count)
                else:
                    reset += 1
                    print 'NOT FOUND MAT: %d'%(reset)
                if count >= 5:
                    print 'MODE 2'
                    mode = 2
                auv.move(drum_pos, cons.AUV_SPEED_FIND_DRUM)

            if mode == 2:
                self.detect('drum')
                appear = self.data.appear
                print '---mode 2---'
                if appear:
                    count += 1
                    reset = 0
                    print 'FOUND DRUM: %d'%(count)
                else:
                    reset += 1
                    print 'NOT FOUND DRUM: %d'%(reset)
                if reset >= 5:
                    count = 0
                    reset = 0
                if count >= 5:
                    print 'MODE 3'
                    count = 0
                    mode = 3
                    switch_mode += 1
                auv.move(drum_pos, cons.AUV_SPEED_FIND_DRUM)

            if mode == 3:
                self.detect('drum')
                appear = self.data.appear
                cx = self.data.cx
                cy = self.data.cy
                area = self.data.area
                print '---mode 3---'
                if not appear:
                    reset += 1
                    print 'NOT FOUND DRUM: %d'%(reset)
                else:
                    reset = 0
                    if cx < 0:
                        auv.move('left', (cons.AUV_SPEED_ADJUST)*abs(cx))
                        drum_pos = 'left'
                    elif cx > 0:
                        auv.move('right', (cons.AUV_SPEED_ADJUST)*abs(cx))
                        drum_pos = 'right'
                    if cy > 0:
                        auv.move('backward', (cons.AUV_SPEED_ADJUST)*abs(cy))
                    elif cy < 0:
                        auv.move('forward', (cons.AUV_SPEED_FIND_DRUM-0.3)*abs(cy))
                    if -cons.VISION_DRUM <= cx <= cons.VISION_DRUM and -cons.VISION_DRUM <= cy <= cons.VISION_DRUM:
                        count += 1
                if count >= 1:
                    mode = 4
                    print 'MODE 4'
                if reset >= 5:
                    mode = 2
                    auv.stop()
                    print 'MODE 2'
                    switch_mode += 1

            if mode == 4:
                print '---mode 4---'
                auv.depthAbs(cons.DRUM_DEPTH+0.5)
                #auv.turnRelative(cons.FLARE_DEGREE)
                auv.driveX(cons.DRUM_DROP_BALL)
                auv.gripper('on')
                auv.gripper('on')
                rospy.sleep(2)
                auv.gripper('off')
                auv.gripper('off')

            if switch_mode > 5:
                print 'ABORT DRUM MISSION'
                mode = 4
                print 'DROP THE BALL'

        print 'completed drop ball'

if __name__=='__main__':
    mission_drum = Drum()
    mission_drum.run()
