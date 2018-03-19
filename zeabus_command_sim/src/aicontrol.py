#!/usr/bin/python2#!/usr/bin/python2

import rospy, math, tf
import constants as cons
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool

class AIControl:

    def __init__(self):
        rospy.init_node('robot_node')
        self.speed = Twist()
        self.pose = Pose()

        self.auv_state = [0, 0, 0, 0, 0, 0]

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_rel_yaw = rospy.Publisher('/fix/rel/yaw', Float64, queue_size=10)
        self.pub_abs_yaw = rospy.Publisher('/fix/abs/yaw', Float64, queue_size=10)
        self.pub_abs_depth = rospy.Publisher('/fix/abs/depth', Float64, queue_size=10)

        rospy.Subscriber('/auv/state', Odometry, self.getState)

    def move(self, direction, speed):
        self.stop()

        print ('Move %s'%(direction))
        if direction == 'left':
            self.speed.linear.y = speed
        elif direction == 'right':
            self.speed.linear.y = -speed
        elif direction == 'forward':
            self.speed.linear.x = speed
        elif direction == 'backward':
            self.speed.linear.x = -speed
        elif direction == 'up':
            self.speed.linear.z = speed
        elif direction == 'down':
            self.speed.linear.z = -speed

        for _ in range(3):
            self.pub_vel.publish(self.speed)

    def moveToPos(self, x, y, z):

        destination_degree =  math.degrees(math.atan2(y - self.auv_state[1], x - self.auv_state[0]))
        current_degree = math.degrees(self.auv_state[5])
        turning_degree = destination_degree - current_degree

        distance = math.sqrt((x - self.auv_state[0])**2 + (y - self.auv_state[1])**2)
        old_distance = distance
        if distance >= 0.3:
            self.turnRelative(turning_degree)

        print turning_degree
        decreasing = True

        while distance >= 0.5 and decreasing:

            current_degree = math.degrees(self.auv_state[5])
            turning_degree = destination_degree - current_degree
            if abs(turning_degree) >= 0.5 and distance <= 20:
                self.turnRelative(turning_degree)
            if abs(turning_degree) >= 0.1 and distance <= 3:
                self.turnRelative(turning_degree)

            self.move('forward', cons.AUV_SPEED * distance)
            distance = math.sqrt((x - self.auv_state[0])**2 + (y - self.auv_state[1])**2)
            decreasing = (old_distance > distance )
            old_distance = distance
            print ('distance: %f')%(distance)

        self.stop()
        print self.auv_state

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

    def turnRelative(self, angle):
        self.stop()
        print 'turning'

        if angle > 180:
            angle %= 360
            angle -= 360
        elif angle < -180:
            angle %= 360
            angle += 360

        rand = math.radians(angle)
        for _ in range(3):
            self.pub_rel_yaw.publish(rand)
        print angle
        delay = abs(5.3*angle/180)
        rospy.sleep(delay)

        self.stop()
        print 'stop'

    def turnAbs(self, angle):
        self.stop()

        print 'turning'
        rand = math.radians(angle)
        for _ in range(3):
            self.pub_abs_yaw.publish(rand)

        rospy.sleep(5.2)
        self.stop()
        print 'stop'

    def depthAbs(self, dep):
        self.stop()
        print 'moving'
        self.pub_abs_depth.publish(dep)

        rospy.sleep(2)
        self.stop()
        print 'stop'

    def isCenterX(self, x, angle):
        if abs(x) >= 0.5:
            if(x > 0):
                self.move('left', cons.AUV_SPEED*x)
            elif(x < 0):
                self.move('right', cons.AUV_SPEED*abs(x))
            return False

        if(abs(angle) > 0.5):
            self.turnRelative(angle)
            return False
        return True

    def isFail(self,count):
        if count>0:
            return False
        return True
    
    def stop(self):
        rospy.sleep(0.5)
        self.speed.linear.x = 0
        self.speed.linear.y = 0
        self.speed.linear.z = 0

        self.speed.angular.x = 0
        self.speed.angular.y = 0
        self.speed.angular.z = 0

        for _ in range(3):
            self.pub_vel.publish(self.speed)

if __name__=='__main__':
    zeabus_robot = AIControl()
    # count = 5
    # while not rospy.is_shutdown() and not zeabus_robot.isFail(count):
    #     zeabus_robot.move('',1)
    #     rospy.loginfo('\nx: %f\ny: %f\nz: %f'%(zeabus_robot.auv_state[0], zeabus_robot.auv_state[1], zeabus_robot.auv_state[2]))
    #     count-=1

