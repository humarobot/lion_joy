#!/usr/bin/env python3
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
tw = Twist()

def callback(data:Joy):
    # print(data.axes[0]) 
    tw.linear.x = 0.5*data.axes[1]
    tw.linear.y = 0.5*data.axes[0]
    tw.angular.z = data.axes[2]
    



if __name__ == '__main__':
    try:
        rospy.init_node('lion_joy', anonymous=True)
        sub = rospy.Subscriber('joy',Joy,callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub.publish(tw) 
            rate.sleep()
    except rospy.ROSInterruptException:
        pass