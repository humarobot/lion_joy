#!/usr/bin/env python2
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule
from geometry_msgs.msg import PoseStamped
from ocs2_msgs.msg import mpc_observation
from std_msgs.msg import Bool,Int32
import tf
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
hand_publisher = rospy.Publisher('/hand_state',Bool,queue_size=10)
demonstrate_publisher = rospy.Publisher('/demonstrator',Int32,queue_size=10)
tw = Twist()

hand_state = Bool()
demonstrate_command = Int32()

init_flag = True
stand_up_enter_flag = False
stand_up_flag = False
sit_down_enter_flag = False
sit_down_flag = True
stance_flag = True
trot_flag = False
gait_switch_flag = False
body_twist_flag = False
hand_close_flag = False
recording_flag = False
replaying_flag = False

x_obs = 0.0
y_obs = 0.0
z_obs = 0.0
rpy_obs = [0.0,0.0,0.0]

static_walk = mode_schedule()
static_walk.eventTimes = [0.0,0.3,0.6,0.9,1.2]
static_walk.modeSequence = [13,7,14,11]

trot = mode_schedule()
trot.eventTimes = [0.0,0.35,0.7]
trot.modeSequence = [9,6]

stance = mode_schedule()
stance.eventTimes = [0.0,0.5]
stance.modeSequence = [15]

sit_down_pose = PoseStamped()
sit_down_pose.header.frame_id = "odom"
sit_down_pose.pose.position.x = 0.0
sit_down_pose.pose.position.y = 0.0
sit_down_pose.pose.position.z = 0.1
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

target_pose = PoseStamped()
target_pose.header.frame_id = "odom"
target_pose.pose.position.x = 0.0
target_pose.pose.position.y = 0.0
target_pose.pose.position.z = 0.5
target_pose.pose.orientation.x = 0.0
target_pose.pose.orientation.y = 0.0
target_pose.pose.orientation.z = 0.0
target_pose.pose.orientation.w = 1.0
target_rpy = [0.0,0.0,0.0]

total_vel = 0.0
last_total_vel = 0.0


def callback_joy(data):
    # print(data.axes[0]) 
    global stand_up_enter_flag,stand_up_flag
    global sit_down_enter_flag,sit_down_flag
    global trot_flag,stance_flag,gait_switch_flag
    global vel_update_flag
    global body_twist_flag
    global hand_close_flag
    global recording_flag,replaying_flag
    if(data.buttons[LB]==1 and sit_down_flag and stance_flag):
        print("LB")
        stand_up_enter_flag = True
    elif(data.buttons[LB]==1 and stand_up_flag and stance_flag):
        print("LB")
        sit_down_enter_flag = True
    #if RB button was pressed, switch between stance and trot
    if(data.buttons[RB]==1 and stance_flag and stand_up_flag):
        trot_flag=True
        stance_flag=False
        gait_switch_flag=True
    elif(data.buttons[RB]==1 and trot_flag and stand_up_flag):
        stance_flag=True
        trot_flag=False
        gait_switch_flag=True
    #if A button was pressed, entering body twist mode
    if(data.buttons[A]==1 and stance_flag and stand_up_flag):
        # print("A")
        body_twist_flag=True
    elif(data.buttons[A]==0 and stance_flag and stand_up_flag):
        # print("release A")
        body_twist_flag=False
    # B button control hands
    if(data.buttons[B]==1 and hand_close_flag==False):
        print("hand closing")
        hand_close_flag = True
    elif(data.buttons[B]==1 and hand_close_flag==True):
        print("hand opening")
        hand_close_flag = False
    # X button control recording
    if(data.buttons[X]==1 and recording_flag==False):
        print("start recording")
        demonstrate_command.data = 1
        demonstrate_publisher.publish(demonstrate_command)
        recording_flag = True
    elif(data.buttons[X]==1 and recording_flag==True):
        print("stop recording")
        demonstrate_command.data = 2
        demonstrate_publisher.publish(demonstrate_command)
        recording_flag = False
    # Y button control replaying
    if(data.buttons[Y]==1 and replaying_flag==False and recording_flag==False):
        print("start replaying")
        demonstrate_command.data = 3
        demonstrate_publisher.publish(demonstrate_command)
        replaying_flag = True
    elif(data.buttons[Y]==1 and replaying_flag==True and recording_flag==False):
        print("stop replaying")
        demonstrate_command.data = 4
        demonstrate_publisher.publish(demonstrate_command)
        replaying_flag = False
    tw.linear.x = 0.5*data.axes[1]
    tw.linear.y = 0.5*data.axes[0]
    tw.angular.z = data.axes[2]

def callback_state(data):
    global x_obs,y_obs,z_obs,rpy_obs
    global sit_down_flag,stand_up_flag,init_flag
    x_obs = data.state.value[6]
    y_obs = data.state.value[7]    
    z_obs = data.state.value[8]
    rpy_obs[0] = data.state.value[9]
    if(init_flag):
        if(z_obs<0.1):
            sit_down_flag = True
            stand_up_flag = False
        elif(z_obs>0.4):
            sit_down_flag = False
            stand_up_flag = True
        init_flag = False
    # rpy_obs[1] = data.state.value[10]
    # rpy_obs[2] = data.state.value[11]
    # print(rpy_obs)


if __name__ == '__main__':
    try:
        rospy.init_node('lion_joy')
        joy_subscriber = rospy.Subscriber('joy',Joy,callback_joy)
        state_subscriber = rospy.Subscriber('legged_robot_mpc_observation',mpc_observation,callback_state)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #TODO get current pose

            if stand_up_enter_flag and not stand_up_flag and stance_flag:
                print("standing up")
                sit_down_pose.pose.position.z = sit_down_pose.pose.position.z + 0.05
                sit_down_pose.pose.position.x = target_pose.pose.position.x
                sit_down_pose.pose.position.y = target_pose.pose.position.y
                #rpy_obs to quaternion
                #restrict rpy_obs[0] to be in [0,2*pi]
                if rpy_obs[0] < 0:
                    rpy_obs[0] = rpy_obs[0] + 2*math.pi
                elif rpy_obs[0] > 2*math.pi:
                    rpy_obs[0] = rpy_obs[0] - 2*math.pi
                quat = tf.transformations.quaternion_from_euler(rpy_obs[0],rpy_obs[1],rpy_obs[2])
                sit_down_pose.pose.orientation.x = quat[0]
                sit_down_pose.pose.orientation.y = quat[1]
                sit_down_pose.pose.orientation.z = quat[2]
                sit_down_pose.pose.orientation.w = quat[3]
                target_publisher.publish(sit_down_pose)
                if sit_down_pose.pose.position.z >= 0.5:
                    stand_up_enter_flag = False
                    stand_up_flag = True
                    sit_down_flag = False
                    sit_down_pose.pose.position.z = 0.1
                    print("stand up finished")
            elif sit_down_enter_flag and not sit_down_flag and stance_flag:
                print("sitting down")
                stand_up_pose.pose.position.z = stand_up_pose.pose.position.z - 0.05
                stand_up_pose.pose.position.x = target_pose.pose.position.x
                stand_up_pose.pose.position.y = target_pose.pose.position.y
                quat = tf.transformations.quaternion_from_euler(rpy_obs[0],rpy_obs[1],rpy_obs[2])
                stand_up_pose.pose.orientation.x = quat[0]
                stand_up_pose.pose.orientation.y = quat[1]
                stand_up_pose.pose.orientation.z = quat[2]
                stand_up_pose.pose.orientation.w = quat[3]
                target_publisher.publish(stand_up_pose)
                if stand_up_pose.pose.position.z <= 0.1:
                    sit_down_enter_flag = False
                    sit_down_flag = True
                    stand_up_flag = False
                    stand_up_pose.pose.position.z = 0.5
                    print("sit down finished")

            if trot_flag and gait_switch_flag:
                print("trot mode")
                mode_publisher.publish(trot)
                gait_switch_flag=False
            elif stance_flag and gait_switch_flag:
                print("stance mode")
                mode_publisher.publish(stance)
                gait_switch_flag=False

            if body_twist_flag:
                yaw = tw.linear.y
                pitch = 0.5*tw.linear.x
                stand_up_pose.pose.position.z = stand_up_pose.pose.position.z
                stand_up_pose.pose.position.x = x_obs
                stand_up_pose.pose.position.y = y_obs
                quat = tf.transformations.quaternion_from_euler(yaw,pitch,0.0)
                stand_up_pose.pose.orientation.x = quat[0]
                stand_up_pose.pose.orientation.y = quat[1]
                stand_up_pose.pose.orientation.z = quat[2]
                stand_up_pose.pose.orientation.w = quat[3]
                target_publisher.publish(stand_up_pose)
            
            if(trot_flag):
                # vel_publisher.publish(tw) 
                target_pose.pose.position.x = target_pose.pose.position.x + tw.linear.x*0.1
                target_pose.pose.position.y = target_pose.pose.position.y + tw.linear.y*0.1
                target_rpy[0] = target_rpy[0] + tw.angular.z*0.1
                quat = tf.transformations.quaternion_from_euler(target_rpy[0],target_rpy[1],target_rpy[2])
                target_pose.pose.orientation.x = quat[0]
                target_pose.pose.orientation.y = quat[1]
                target_pose.pose.orientation.z = quat[2]
                target_pose.pose.orientation.w = quat[3]
                target_publisher.publish(target_pose)
            
            if(hand_close_flag):
                hand_state.data=True
                hand_publisher.publish(hand_state)
            else:
                hand_state.data=False
                hand_publisher.publish(hand_state)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass