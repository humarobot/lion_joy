#!/usr/bin/env python3
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule
import math

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
mode_pub = rospy.Publisher('legged_robot_mpc_mode_schedule',mode_schedule, queue_size=10)
tw = Twist()

static_walk = mode_schedule()
static_walk.eventTimes = [0.0,0.3,0.6,0.9,1.2]
static_walk.modeSequence = [13,7,14,11]

trot = mode_schedule()
trot.eventTimes = [0.0,0.3,0.6]
trot.modeSequence = [9,6]

stance = mode_schedule()
stance.eventTimes = [0.0,0.5]
stance.modeSequence = [15]

total_vel = 0.0
last_total_vel = 0.0


def callback(data:Joy):
    # print(data.axes[0]) 
    tw.linear.x = 0.5*data.axes[1]
    tw.linear.y = 0.5*data.axes[0]
    tw.angular.z = data.axes[2]


if __name__ == '__main__':
    try:
        rospy.init_node('lion_joy')
        sub = rospy.Subscriber('joy',Joy,callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            total_vel = math.sqrt(tw.linear.x**2+tw.linear.y**2+tw.angular.z**2)

            if total_vel>0.03 and last_total_vel<=0.03:
                mode_pub.publish(trot)
            elif total_vel<0.03 and last_total_vel>=0.03:
                mode_pub.publish(stance)
            last_total_vel = total_vel

            pub.publish(tw) 
            rate.sleep()
    except rospy.ROSInterruptException:
        pass