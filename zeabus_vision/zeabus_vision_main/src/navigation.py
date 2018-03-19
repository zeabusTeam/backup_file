#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf
import math
position = [0, 0, 0, 0, 0, 0, 0]


def twopi(rad):
    if rad <= 0:
        return abs(rad)
    else:
        return 2 * math.pi - rad


def get_position(data):
    global position

    pose = data.pose.pose
    temp = (pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
    angular = tf.transformations.euler_from_quaternion(temp)
    position[0] = pose.position.x
    position[1] = pose.position.y
    position[2] = pose.position.z
    position[3] = angular[0]
    position[4] = angular[1]
    position[5] = angular[2]
    position[6] = math.degrees(position[5])


def drive(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    pubTwist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    t = Twist()
    t.linear.x = x
    t.linear.y = y
    t.linear.z = z
    t.angular.x = roll
    t.angular.y = pitch
    t.angular.z = yaw
    for i in xrange(3):
        print t
        pubTwist.publish(t)
        rospy.sleep(0.05)


def turn_yaw_rel(degree):
    turnYawRel = rospy.Publisher('/fix/rel/yaw', Float64, queue_size=10)
    rad = math.radians(degree)
    rad = Float64(rad)
    turnYawRel.publish(rad)
    rospy.sleep(6)
    print ('turn yaw relative: ', rad)


def turn_yaw_abs(degree):
    turnYawAbs = rospy.Publisher(
        '/fix/abs/yaw', Float64, queue_size=10)
    rad = math.radians(degree)
    rad = Float64(rad)
    for i in xrange(5):
        turnYawAbs.publish(rad)
        rospy.sleep(0.05)
    print ('turn yaw absolute: ', rad)


def go_to_xy(x=None, y=None):
    global position
    print ('goto')
    print (x, y)
    if x is None:
        x = position[0]
    if y is None:
        y = position[1]
    #               X
    #               |
    #               |
    # -y ----------------------- y
    #               |
    #               |
    #               -x
    while not rospy.is_shutdown():
        begin_x = position[0]
        begin_y = position[1]

        delta_x = x - begin_x
        delta_y = -y - begin_y

        begin_rad = math.atan2(delta_y, delta_x)
        begin_deg = math.degrees(begin_rad)
        if begin_deg < 0:
            begin_deg += 360
        begin_deg += 90
        begin_deg %= 360
        print 'begin deg'
        print begin_deg
        rad = position[5]
        # print rad
        deg = math.degrees(rad)
        if begin_deg < 0:
            begin_deg += 360
        print 'deg'
        print deg
        res_deg = [deg - begin_deg - 360, deg -
                   begin_deg, deg - begin_deg + 360]
        res_deg = min(res_deg, key=abs)
        print('res degree')
        print(res_deg)
        turn_yaw_rel(res_deg)
        print 'drive'
        drive(0.5, 0, 0, 0, 0, 0)
        rospy.sleep(1)
        # drive(0.0, 0, 0, 0, 0, 0)
        #
        print 'fin drive'
        if ((begin_x - x)**2 + (begin_y - y)**2 <= (0.2**2)):
            print('stop')
            drive(0, 0, 0, 0, 0, 0)
            rospy.sleep(2)
            break
        print position
    print ('finish')
    print position

if __name__ == '__main__':
    rospy.init_node('navigation')
    # rospy.Subscriber('/auv/state', Odometry, get_position)
    rospy.Subscriber('/syrena/state', Odometry, get_position)

    # pubTwist = rospy.Publisher('/zeabus/cmd_vel', Twist, queue_size=10)
    drive(0, 0, 0, 0, 0, 0)
    # while not rospy.is_shutdown():
    #     rospy.sleep(0.1)
    #     print position[6]
    turn_yaw_abs(0)
    rospy.sleep(1)
    # print 'start'
    print position
    # go_to_xy(0, 0)
