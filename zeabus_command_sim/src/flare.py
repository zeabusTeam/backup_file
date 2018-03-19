#!/usr/bin/python2
import rospy
from std_msgs.msg import String, Float64
from aicontrol import AIControl
import constants as cons
# import pinger as follow_pinger

class Flare(object):
    def __init__(self):
        print 'now do flare'
        
        ## vision service 

        gate_srv = 'gate'
        rospy.wait_for_service(flare_srv)
        print 'flare_srv start'
        self.detect_gate = rospy.ServiceProxy(flare_srv)
        self.check_flare = False
        self.pos_center = self.detect_gate.data_gate.pos_flare

        self.aicontrol = AIControl()

    def bump(self):
        auv = self.aicontrol
                
        # print "following pinger"
        count = cons.FLARE_ROLLCHECK
        if not self.check_flare:
            print "cannot found with pinger"
            print "start found gate 2"
            while not rospy.is_shutdown() and not self.check_flare:
                self.check_flare = self.detect_gate.data_gate.check_flare
                if not self.check_flare:
                    auv.turnRelative(-1)
                    count-=1
                else:
                    print "found gate"
                    break
        auv.stop()
        if self.check_flare = False :
            print "cannot found gate"

        print 'move to center'
        count = cons.FLARE_COUNT 
        while not rospy.is_shutdown() and not auv.isFail(count) :
            self.pos_center = self.detect_gate.data_gate.pos_black
            if self.pos_center >= cons.SCREEN_WIDTH/2*0.9 and self.pos_center <= cons.SCREEN_WIDTH/2*1.1:
                print("good aim")
                break
            elif self.pos_center < cons.SCREEN_WIDTH/2*0.95:
                auv.move('right',cons.FLARE_SPEED)
            elif self.pos_center > cons.SCREEN_WIDTH/2*1.05:
                auv.move('left',cons.FLARE_SPEED)
            count -= 1
            ## move to center
        auv.stop()
        print 'at center'

        print "move forward"
        count = cons.FLARE_BUMP        
        print "bump!!"
        while not rospy.is_shutdown() and not self.aicontrol.isFail(count)
            auv.move('forward',cons.FLARE_SPEED)
            count -=1    
        ## bump flare
        auv.stop()
        
if __init__ == '__main__': 
    mission_gate = Flare()
    mission_gate.bump()
    print "finish gate"