#!/usr/bin/python2.7

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_example.srv import vision_srv_gate
from zeabus_example.msg import vision_gate
from aicontrol import AIControl
import constants as cons

class Gate(object):
    def __init__(self):
        self.data = vision_gate()

        print '<===INIT GATE===>'
        self.aicontrol = AIControl()
        rospy.wait_for_service('vision_gate')
        self.detect_gate = rospy.ServiceProxy('vision_gate', vision_srv_gate)

    def detectGate(self):
        self.data = self.detect_gate(String('gate'), String('gate'))
        self.data = self.data.data

    def run(self):
        auv = self.aicontrol
        print '<===DOING GATE===>'
        # - drive depth
        print '<---drive z--->'
        auv.depthAbs(cons.GATE_DEPTH)
        print '<---drive complete--->'
        # --------------

        count = 0
        reset = 0
        mode = 0
        back = 0
        forward = 0
        center = 0

        while not rospy.is_shutdown() and not mode == -1:
            self.detectGate()
            area = self.data.area
            appear = self.data.appear
            cx = self.data.cx+cons.VISION_FRONT_FIX
            pos = self.data.pos
            area = self.data.area

            if mode == 0:
                print '<---get to detect range--->'

                auv.driveX(cons.GATE_FRONT, 0.5)
                mode = 1
                auv.stop()
                print '<---stop in detect range--->'

            if mode == 1:
                print '---mode 1---'
                if appear:
                    count += 1
                    reset = 0
                    print 'FOUND GATE: %d'%(count)
                else:
                    reset += 1
                    print 'NOT FOUND GATE: %d'%(reset)

                if count >= 5:
                    print 'MODE 2'
                    mode = 2
                if reset >= 5:
                    count = 0
                    reset = 0

                if pos == 0 and appear and -cons.ERROR <= cx <= cons.ERROR and count >= 5:
                    mode = 3
                    print '<<CENTER>>'
                    print 'MODE 3'

                auv.move(cons.GATE_POS, cons.AUV_SPEED_FIND_GATE)

            if mode == 2:
                print '----mode 2----'
                print 'POS: %d'%(pos)
                print 'cx: %f'%(cx)
                print 'area: %f'%(area)
                if not appear:
                    count -= 1
                else:
                    count = 10
                    if back == 0 and forward == 0:
                        if pos == -1:
                            auv.move('right', cons.AUV_SPEED_FIND_GATE)
                        elif pos == 1:
                            auv.move('left', cons.AUV_SPEED_FIND_GATE)
                        elif pos == 0:
                            if cx < 0:
                                auv.move('left', cons.AUV_SPEED_ADJUST*abs(cx))
                            elif cx > 0:
                                auv.move('right', cons.AUV_SPEED_ADJUST*abs(cx))

                            if -cons.ERROR <= cx <= cons.ERROR:
                                print '<<CENTER>>'
                                center += 1
                                auv.stop()
                            else:
                                reset += 1
                            if reset >= 4:
                                center = 0
                                reset = 0
                            if center >= 3:
                                mode = 3
                                print 'MODE 3'

                    if area >= 0.01:
                        print 'TOO CLOSE'
                        back += 1
                        forward = 0
                        auv.stop()

                    elif area < 0.005 and area != 0:
                        print 'TOO FAR'
                        forward += 1
                        back = 0
                        auv.stop()
                    else:
                        forward = 0
                        back = 0

                if back > 3:
                    print 'GOING BACK'
                    back = 1
                    forward = 0
                    auv.driveX(-2, 0.5)
                if forward > 3:
                    print 'GOING FORWARD'
                    forward = 1
                    back = 0
                    auv.driveX(0.5, 0.2)
                if count <= 0:
                    print 'MODE 1'
                    mode = 1

            if mode == 3:
                print '----mode 3----'
                if not appear:
                    count -= 1
                    print 'GATE NOT FOUND: %d'%(count)
                    rospy.sleep(2)
                else:
                    count = 0
                    if pos != 0 or -cons.ERROR > cx or cons.ERROR < cx :
                        reset += 1
                        print 'GATE NOT AT CENTER'
                        rospy.sleep(2)
                    elif pos == 0 and -cons.ERROR <= cx <= cons.ERROR:
                        reset = 0
                        count += 1
                        #auv.move('forward', cons.AUV_FULL_SPEED)
                        mode = -1
                    if count >= 3:
                        auv.driveX(8)
                    elif reset >= 10:
                        print 'MODE 2'
                        mode = 2
                        auv.stop()
            auv.stop()
        auv.stop()

if __name__=='__main__':
    mission_gate = Gate()
    mission_gate.run()
    print 'finish gate'
