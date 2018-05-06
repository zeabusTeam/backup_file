#!/usr/bin/python2

import rospy, math, tf
import constants as cons
from zeabus_elec_ros_hardware_interface.srv import IOCommand
from zeabus_controller.msg import point_xy
from zeabus_controller.srv import *
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Bool

class AIControl:

    def __init__(self):
        rospy.init_node('aicontrol_node')
        self.vel = Twist()
        self.pose = Pose()
        self.point = Point()

        self.auv_state = [0, 0, 0, 0, 0, 0]
        '''
        self.at_pos = True
        self.at_yaw = True
        '''
        self.twist = [0, 0, 0, 0, 0, 0]
        rospy.Subscriber('/auv/state', Odometry, self.getState)
        #rospy.Subscriber('/ok_position', Bool, self.checkPos)
        #rospy.Subscriber('/ok_yaw', Bool, self.checkYaw)
        # real
        self.pub_vel = rospy.Publisher('/zeabus/cmd_vel', Twist, queue_size=10)
        # sim
        #self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.pub_position = rospy.Publisher('/cmd_fix_position', Point, queue_size=10)
        self.pub_xy = rospy.Publisher('/fix/abs/xy', point_xy, queue_size=10)

        self.pub_abs_yaw = rospy.Publisher('/fix/abs/yaw', Float64, queue_size=10)
        self.pub_abs_depth = rospy.Publisher('/fix/abs/depth', Float64, queue_size=10)
        print 'doing something'

        '''
        rospy.wait_for_service('io_and_pressure/IO_ON')
        print 'IO_ON'
        rospy.wait_for_service('io_and_pressure/IO_OFF')
        print 'IO_OFF'
        '''
        rospy.wait_for_service('fix_abs_yaw')
        print 'abs_yaw'
        rospy.wait_for_service('fix_rel_xy')
        print 'rel_xy'
        rospy.wait_for_service('fix_abs_xy')
        print 'abs_xy'
        rospy.wait_for_service('fix_abs_depth')
        print 'abs_depth'
        rospy.wait_for_service('ok_position')
        print 'ok_pos'
        rospy.wait_for_service('fix_service')
        print 'fix_service'

        self.srv_io_on = rospy.ServiceProxy('io_and_pressure/IO_ON', IOCommand)
        self.srv_io_off = rospy.ServiceProxy('io_and_pressure/IO_OFF', IOCommand)
        self.srv_abs_yaw = rospy.ServiceProxy('fix_abs_yaw', fix_abs_yaw)
        self.srv_rel_xy = rospy.ServiceProxy('fix_rel_xy', fix_rel_xy)
        self.srv_abs_xy = rospy.ServiceProxy('fix_abs_xy', fix_abs_xy)
        self.srv_abs_depth = rospy.ServiceProxy('fix_abs_depth', fix_abs_depth)
        self.srv_ok_pos = rospy.ServiceProxy('ok_position', ok_position)
        self.srv_full_speed = rospy.ServiceProxy('fix_service', message_service)

    def fullSpeed(self, time):
        print 'WARNING: BE READY TO STOP THE CONTROL NODE, IF ANYTHING GOING WRONG'
        self.srv_full_speed(String("SAUV"))
        rospy.sleep(time)
        self.srv_full_speed(String(""))
        print 'AUV stop'


    def move(self, direction, speed):
        print("speed : %.2f"%(speed))
        self.stop()

        print 'Move %s at speed %f m/s'%(direction, speed)

        if direction == 'left':
            self.vel.linear.y = speed
        elif direction == 'right':
            self.vel.linear.y = -speed
        elif direction == 'forward':
            self.vel.linear.x = speed
        elif direction == 'backward':
            self.vel.linear.x = -speed

        for _ in range(3):
            self.pub_vel.publish(self.vel)

    def moveX(self, speed):
        print 'Move x at speed %f'%(speed)
        self.stop()
        temp = Twist()
        temp.linear.x = speed
        temp.linear.y = 0
        temp.linear.z = 0

        temp.angular.x = 0
        temp.angular.y = 0
        temp.angular.z = 0
        for _ in range(3):
            self.pub_vel.publish(temp)

    def moveY(self, speed):
        print 'Move y at speed %f'%(speed)
        temp = Twist()
        temp.linear.x = 0
        temp.linear.y = speed
        temp.linear.z = 0

        temp.angular.x = 0
        temp.angular.y = 0
        temp.angular.z = 0
        for _ in range(3):
            self.pub_vel.publish(temp)

    def fixXY(self, x, y, err=0):
        print 'Move to (%f, %f)'%(x, y)
        self.srv_abs_xy(x, y)
        while not rospy.is_shutdown() and not self.srv_ok_pos(String('xy'), err).ok:
            rospy.sleep(0.1)

        print 'finish moving'

    def driveX(self, x, err=0):
        print 'doing drive x'
        self.stop()

        self.srv_rel_xy(x, 0)

        while not rospy.is_shutdown() and not self.srv_ok_pos(String('xy'), err).ok:
            rospy.sleep(0.1)

        print 'finish drive x'

    def driveY(self, y, err=0):
        print 'doing drive y'
        self.stop()

        self.srv_rel_xy(0, y)

        while not rospy.is_shutdown() and not self.srv_ok_pos(String('xy'), err).ok:
            rospy.sleep(0.1)

        print 'finish drive y'

    def turnRelative(self, degree, err=0):
        self.stop()
        print 'turning %f'%(degree)
        degree %= 360

        old_degree = math.degrees(self.auv_state[5])
        degree += old_degree

        rand = math.radians(degree)
        '''
        for _ in range(30):
            self.pub_abs_yaw.publish(rand)
        '''
        self.srv_abs_yaw(rand)

        while not rospy.is_shutdown() and not self.srv_ok_pos(String('yaw'), err).ok:
            rospy.sleep(0.4)

        print 'finish turn rel'

    def turnAbs(self, degree, err=0):
        self.stop()
        print 'turning to %f'%(degree)
        rand = math.radians(degree)
        '''
        for _ in range(30):
            self.pub_abs_yaw.publish(rand)
        '''
        self.srv_abs_yaw(rand)

        while not rospy.is_shutdown() and not self.srv_ok_pos(String('yaw'), err).ok:
            rospy.sleep(0.4)

        print 'finish turn abs'

    def depthAbs(self, depth, err=0):
        self.stop()
        print 'move to depth %f'%(depth)
        sent = False
        if not sent:
            sent = True
            self.srv_abs_depth(depth)

        while not rospy.is_shutdown() and not self.srv_ok_pos(String('z'), err).ok:
            rospy.sleep(0.4)

        print 'finish depth abs'

    def stop(self):
        rospy.sleep(0.1)
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0

        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        for _ in range(3):
            self.pub_vel.publish(self.vel)
    '''
    def checkPos(self, data):
        self.at_pos = data.data

    def checkYaw(self, data):
        self.at_yaw = data.data
    '''

    def gripper(self, cmd):
        if cmd == 'on':
            self.srv_io_on(5)
        elif cmd == 'off':
            self.srv_io_off(5)

    def getState(self, data):
        self.pose = data.pose.pose
        pose = self.pose

        temp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler_angular = tf.transformations.euler_from_quaternion(temp)

        self.auv_state[0] = pose.position.x
        self.auv_state[1] = pose.position.y
        self.auv_state[2] = pose.position.z

        self.auv_state[3] = euler_angular[0]
        self.auv_state[4] = euler_angular[1]
        self.auv_state[5] = euler_angular[2]

if __name__=='__main__':
    aicontrol = AIControl()
    aicontrol.driveX(0.82)
