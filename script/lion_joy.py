#!/usr/bin/env python3
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule
from geometry_msgs.msg import PoseStamped
import math

# LB: button 6
# RB: button 7
# LT: button 8
# RT: button 9
# A: button 0
# B: button 1
# X: button 3
# Y: button 4
LB= 6
RB= 7
LT= 8
RT= 9
A= 0
B= 1
X= 3
Y= 4

vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
mode_publisher = rospy.Publisher('legged_robot_mpc_mode_schedule',mode_schedule, queue_size=10)
target_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
tw = Twist()


stand_up_enter_flag = False
stand_up_flag = False
sit_down_enter_flag = False
sit_down_flag = True
stance_flag = True
trot_flag = False
gait_switch_flag = False

static_walk = mode_schedule()
static_walk.eventTimes = [0.0,0.3,0.6,0.9,1.2]
static_walk.modeSequence = [13,7,14,11]

trot = mode_schedule()
trot.eventTimes = [0.0,0.3,0.6]
trot.modeSequence = [9,6]

stance = mode_schedule()
stance.eventTimes = [0.0,0.5]
stance.modeSequence = [15]

sit_down_pose = PoseStamped()
sit_down_pose.header.frame_id = "odom"
sit_down_pose.pose.position.x = 0.0
sit_down_pose.pose.position.y = 0.0
sit_down_pose.pose.position.z = 0.0
sit_down_pose.pose.orientation.x = 0.0
sit_down_pose.pose.orientation.y = 0.0
sit_down_pose.pose.orientation.z = 0.0
sit_down_pose.pose.orientation.w = 1.0

stand_up_pose = PoseStamped()
stand_up_pose.header.frame_id = "odom"
stand_up_pose.pose.position.x = 0.0
stand_up_pose.pose.position.y = 0.0
stand_up_pose.pose.position.z = 0.5
stand_up_pose.pose.orientation.x = 0.0
stand_up_pose.pose.orientation.y = 0.0
stand_up_pose.pose.orientation.z = 0.0
stand_up_pose.pose.orientation.w = 1.0

total_vel = 0.0
last_total_vel = 0.0


def callback_joy(data:Joy):
    # print(data.axes[0]) 
    global stand_up_enter_flag
    global sit_down_enter_flag
    global trot_flag,stance_flag,gait_switch_flag
    if(data.buttons[LB]==1 and sit_down_flag):
        print("LB")
        stand_up_enter_flag = True
    elif(data.buttons[LB]==1 and stand_up_flag):
        print("LB")
        sit_down_enter_flag = True
    #if RB button was pressed, switch between stance and trot
    if(data.buttons[RB]==1 and stance_flag):
        trot_flag=True
        stance_flag=False
        gait_switch_flag=True
    elif(data.buttons[RB]==1 and trot_flag):
        stance_flag=True
        trot_flag=False
        gait_switch_flag=True

    tw.linear.x = 0.5*data.axes[1]
    tw.linear.y = 0.5*data.axes[0]
    tw.angular.z = data.axes[2]


if __name__ == '__main__':
    try:
        rospy.init_node('lion_joy')
        sub = rospy.Subscriber('joy',Joy,callback_joy)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if stand_up_enter_flag and not stand_up_flag:
                print("standing up")
                sit_down_pose.pose.position.z = sit_down_pose.pose.position.z + 0.05
                target_publisher.publish(sit_down_pose)
                if sit_down_pose.pose.position.z >= 0.5:
                    stand_up_enter_flag = False
                    stand_up_flag = True
                    sit_down_flag = False
                    sit_down_pose.pose.position.z = 0.0
                    print("stand up finished")
            elif sit_down_enter_flag and not sit_down_flag:
                print("sitting down")
                stand_up_pose.pose.position.z = stand_up_pose.pose.position.z - 0.05
                target_publisher.publish(stand_up_pose)
                if stand_up_pose.pose.position.z <= 0.0:
                    sit_down_enter_flag = False
                    sit_down_flag = True
                    stand_up_flag = False
                    stand_up_pose.pose.position.z = 0.5
                    print("sit down finished")

            if trot_flag and gait_switch_flag:
                print("trot")
                mode_publisher.publish(trot)
                gait_switch_flag=False
            elif stance_flag and gait_switch_flag:
                print("stance")
                mode_publisher.publish(stance)
                gait_switch_flag=False

            # total_vel = math.sqrt(tw.linear.x**2+tw.linear.y**2+tw.angular.z**2)
            # if total_vel>0.03 and last_total_vel<=0.03:
            #     mode_publisher.publish(trot)
            # elif total_vel<0.03 and last_total_vel>=0.03:
            #     mode_publisher.publish(stance)
            # last_total_vel = total_vel
            # vel_publisher.publish(tw) 
            rate.sleep()
    except rospy.ROSInterruptException:
        pass