#!/usr/bin/env python
from modbus_ascii_ros.srv import SendCommand
import rospy

if __name__ == "__main__":
    rospy.sleep(1.0);
   # rospy.wait_for_service('zeabus_modbus_master/set_regulator_main')
    #rospy.wait_for_service('zeabus_modbus_master/set_regulator_aux')
    #thruster = rospy.ServiceProxy('zeabus_modbus_master/set_regulator_aux',SendCommand)
   # sensor = rospy.ServiceProxy('zeabus_modbus_master/set_regulator_main',SendCommand)
    #thruster([1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0])
    #sensor([0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0])

