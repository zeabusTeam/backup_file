#!/usr/bin/env python
import rospy
from modbus_ascii_ros.msg import Switch
from modbus_ascii_ros.srv import SendCommand
class shutdown:
    def __init__(self):
        self.ever_turned_on = False
        rospy.init_node("shutdown_node" , disable_signals = True)
        rospy.Subscriber("/switch/data" , Switch , self.callback)
        #self.thruster = rospy.ServiceProxy('zeabus_modbus_master/set_regulator_aux',SendCommand)
        #self.sensor = rospy.ServiceProxy('zeabus_modbus_master/set_regulator_main',SendCommand)
        self.stack = 0
    def callback(self,msg):
        if not msg.motor_switch:
            self.stack += 1
        else:
            self.ever_turned_on = True
            self.stack = 0
        if self.stack >= 10 and self.ever_turned_on:
            #self.thruster([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            #self.sensor([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            rospy.sleep(0.2)
            rospy.signal_shutdown("control is shutdown")




if __name__ == "__main__":
    #rospy.sleep(2)
    #rospy.signal_shutdown("xxxxxxxxx")
    x = shutdown()
    rospy.spin()
