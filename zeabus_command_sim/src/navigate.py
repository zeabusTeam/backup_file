#!/usr/bin/python2
import rospy
from std_msgs.msg import String, Float64 ,Bool
from zeabus_example.msg import vision_gate
from aicontrol import AIControl
import constants as cons

class Gate(object):
    def __init__(self):
        self.data = vision_gate()
    #    rospy.init_node('mission_gate', anonymous=True)

        print 'now do gate'
        
        ## vision service 

        # print 'gate_srv start'    
        # self.detect = self.detect_gate.self.data.gate
        self.aicontrol = AIControl()

    # def run(self):
    #     auv = self.aicontrol
    #     print "start drive_z"
    #     auv.depthAbs(cons.AUV_DEPTH)
    #     print "drive z complete"
    #     rospy.loginfo('x: %f\ny: %f\nz: %f'%(auv.auv_state[0], auv.auv_state[1], auv.auv_state[2]))
        
    #     print "start find gate"
    #     while not self.check_gate:
    #         self.check_gate = self.detect_gate.self.data.gate.check
    #         self.aicontrol.move('forward',cons.AUV_SPEED)
    #     print "found gate"
    #     self.aicontrol.stop()
    #     ## found gate    
                
    #     print "start turning"     
    #     count = cons.GATE_COUNT
    #     self.green_old = self.red_old = 0
                
    #     while not rospy.is_shutdown() and not self.aicontrol.isFail(count):
    #         count -= 1    
    #         if self.area_red > self.area_green or self.area_red == None:
    #             auv.turnRelative(1)
    #             if self.area_red == None and green_old > self.area_green*cons.GATE_POLE:
    #                 break
    #             elif self.area_red <= self.area_green*cons.GATE_POLE:
    #                 break               
    #             self.green_old = self.area_green                
    #         elif self.area_red < self.area_green or self.area_green == None:
    #             auv.turnRelative(-1)
    #             red_old = 0
    #             if self.area_green == None and red_old > self.area_red*cons.GATE_POLE:
    #                 break          
    #             elif self.area_red*cons.GATE_POLE >= self.area_green:
    #                 break
    #             red_old = self.area_red               
    #         ## rotate to gate
    #     auv.stop()
    #     print "turning complete"

    #     print "find center"
    #     count = cons.GATE_COUNT        
    #     while not rospy.is_shutdown() and not self.aicontrol.isFail(count) :
    #         self.pos_center = self.detect_gate.self.data.gate.pos_black
    #         if self.pos_center >= cons.SCREEN_WIDTH/2*0.9 and self.pos_center <= cons.SCREEN_WIDTH/2*1.1:
    #             print("good aim")
    #             break
    #         elif self.pos_center < cons.SCREEN_WIDTH/2*0.9:
    #             auv.move('right',cons.AUV_SPEED)
    #         elif self.pos_center > cons.SCREEN_WIDTH/2*1.1:
    #             auv.move('left',cons.AUV_SPEED)
    #         count -= 1
    #         ## move to center
    #     print "find center complete"

    #     auv.move('forward',cons.AUV_SPEED)
    #     ## elapse gate
    #     auv.stop()

    def getData(self, data):
        self.data = data

    def run2(self):
        auv = self.aicontrol
        gate_found = self.data.check_top or self.data.check_l or self.data.check_r
        print "start drive_z"
        auv.depthAbs(cons.GATE_DEPTH)
        print "drive z complete"
        rospy.loginfo('x: %f\ny: %f\nz: %f'%(auv.auv_state[0], auv.auv_state[1], auv.auv_state[2]))

        count = cons.GATE_FRONT
        while not rospy.is_shutdown() and not auv.isFail(count):
            count -=1
            #auv.moveToPos(3.5, 0, 0)
            auv.move('forward', 0.2)
        auv.stop()
        print "stop at front gate"

        print "start find gate"  
        while not rospy.is_shutdown() and not gate_found:
            gate_found = self.data.check_top or self.data.check_l or self.data.check_r
            auv.move(cons.GATE_POS,cons.AUV_SPEED)
            print gate_found
        auv.stop()
        print "found gate"

        print 'move to center'
        count = cons.GATE_COUNT 
        while not rospy.is_shutdown() and not auv.isFail(count) :
            self.pos_center = self.data.top_x
<<<<<<< HEAD
            if self.pos_center >= cons.SCREEN_WIDTH/2*0.85 and self.pos_center <= cons.SCREEN_WIDTH/2*1.15:
                print("good aim")
                auv.stop()
                break
            elif self.pos_center > cons.SCREEN_WIDTH/2*0.85:
                auv.move('right',cons.AUV_SPEED*abs((self.pos_center - cons.SCREEN_WIDTH/2)/cons.SCREEN_WIDTH))
            elif self.pos_center < cons.SCREEN_WIDTH/2*1.15:
=======
            if self.pos_center >= cons.SCREEN_WIDTH/2*0.87 and self.pos_center <= cons.SCREEN_WIDTH/2*1.13:
                print("good aim")
                auv.stop()
                break
            elif self.pos_center > cons.SCREEN_WIDTH/2*0.87:
                auv.move('right',cons.AUV_SPEED*abs((self.pos_center - cons.SCREEN_WIDTH/2)/cons.SCREEN_WIDTH))
            elif self.pos_center < cons.SCREEN_WIDTH/2*1.13:
>>>>>>> 46b65bbd282e3bb36a7cbe18869dae817180b51f
                auv.move('left',cons.AUV_SPEED*abs((self.pos_center - cons.SCREEN_WIDTH/2)/cons.SCREEN_WIDTH))
            count -= 1
        auv.stop()
        ## move to center
        print 'at center'

        count = cons.GATE_ELAPSE
<<<<<<< HEAD
        while not rospy.is_shutdown() and count > 0:
=======
        while not rospy.is_shutdown() and not auv.isFail(count):
>>>>>>> 46b65bbd282e3bb36a7cbe18869dae817180b51f
            auv.move('forward',cons.AUV_SPEED)
            #print self.data
            if self.pos_center <= cons.SCREEN_WIDTH/2*0.95 or self.pos_center >= cons.SCREEN_WIDTH/2*1.05:
                print 'gate is not at center.'
                while not rospy.is_shutdown():
                    self.pos_center = self.data.top_x
                    if self.pos_center >= cons.SCREEN_WIDTH/2*0.9 and self.pos_center <= cons.SCREEN_WIDTH/2*1.1:
                        print("good aim")
                        auv.stop()
                        break
                    elif self.pos_center > cons.SCREEN_WIDTH/2*0.9:
                        auv.move('right',cons.AUV_SPEED*abs((self.pos_center - cons.SCREEN_WIDTH/2)/cons.SCREEN_WIDTH))
                        print 'speed: %f'%(cons.AUV_SPEED*abs((self.pos_center - cons.SCREEN_WIDTH/2)/cons.SCREEN_WIDTH))
                    elif self.pos_center < cons.SCREEN_WIDTH/2*1.1:
                        auv.move('left',cons.AUV_SPEED*abs((self.pos_center - cons.SCREEN_WIDTH/2)/cons.SCREEN_WIDTH))
                        print 'speed: %f'%(cons.AUV_SPEED*abs((self.pos_center - cons.SCREEN_WIDTH/2)/cons.SCREEN_WIDTH))
<<<<<<< HEAD
            if self.data.right_area == 0 or self.data.left_area == 0 or self.data.left_area == 0 :
=======
            # if self.data.right_area == 0 or self.data.left_area == 0 or self.data.left_area == 0 :
            if not gate_found :
>>>>>>> 46b65bbd282e3bb36a7cbe18869dae817180b51f
                print 'gate not found'
                count -=1
        auv.stop()
        print "elapse gate"

if __name__ == '__main__':
    mission_gate = Gate()
    #mission_gate.run()
    rospy.Subscriber('/vision_mission_gate', vision_gate, mission_gate.getData)
    mission_gate.run2()
    print "finish gate"
