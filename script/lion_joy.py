#!/usr/bin/env python3
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule
from geometry_msgs.msg import PoseStamped
from ocs2_msgs.msg import mpc_observation
from std_msgs.msg import Bool
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
LB = 6
RB = 7
LT = 8
RT = 9
A = 0
B = 1
X = 3
Y = 4

vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
mode_publisher = rospy.Publisher("legged_robot_mpc_mode_schedule", mode_schedule, queue_size=10)
target_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
execute_publisher = rospy.Publisher("/execute_traj", Bool, queue_size=10)

exec = Bool()
exec.data = False

tw = Twist()

init_flag = True
stand_up_enter_flag = False
stand_up_flag = False
sit_down_enter_flag = False
sit_down_flag = True
stance_flag = True
trot_flag = False
gait_switch_flag = False
auto_mode_flag = False
# 鸡头模式标志
chicken_head_mode_flag = False
chicken_head_enter_flag = False
chicken_head_exit_flag = False

x_obs = 0.0
y_obs = 0.0
z_obs = 0.0
rpy_obs = [0.0, 0.0, 0.0]

static_walk = mode_schedule()
static_walk.eventTimes = [0.0, 0.3, 0.6, 0.9, 1.2]
static_walk.modeSequence = [13, 7, 14, 11]

trot = mode_schedule()
trot.eventTimes = [0.0, 0.3, 0.6]
trot.modeSequence = [9, 6]

stance = mode_schedule()
stance.eventTimes = [0.0, 0.5]
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
target_rpy = [0.0, 0.0, 0.0]

# 鸡头模式用的姿态信息
chicken_head_pose = PoseStamped()
chicken_head_pose.header.frame_id = "odom"
chicken_head_pose.pose.position.x = 0.0
chicken_head_pose.pose.position.y = 0.0
chicken_head_pose.pose.position.z = 0.5  # 默认高度0.5m
chicken_head_pose.pose.orientation.x = 0.0
chicken_head_pose.pose.orientation.y = 0.0
chicken_head_pose.pose.orientation.z = 0.0
chicken_head_pose.pose.orientation.w = 1.0
chicken_head_rpy = [0.0, 0.0, 0.0]  # [yaw, roll, pitch]
chicken_head_base_yaw = 0.0  # 记录进入鸡头模式时的基准yaw值

total_vel = 0.0
last_total_vel = 0.0


def callback_joy(data: Joy):
    # print(data.axes[0])
    global stand_up_enter_flag, stand_up_flag
    global sit_down_enter_flag, sit_down_flag
    global trot_flag, stance_flag, gait_switch_flag
    global vel_update_flag
    global auto_mode_flag
    global exec
    global chicken_head_mode_flag, chicken_head_enter_flag, chicken_head_exit_flag
    global chicken_head_pose, chicken_head_rpy

    if data.buttons[LB] == 1 and sit_down_flag and stance_flag:
        print("LB")
        stand_up_enter_flag = True
    elif data.buttons[LB] == 1 and stand_up_flag and stance_flag:
        print("LB")
        sit_down_enter_flag = True
    # if RB button was pressed, switch between stance and trot
    if data.buttons[RB] == 1 and stance_flag and stand_up_flag:
        trot_flag = True
        stance_flag = False
        gait_switch_flag = True
    elif data.buttons[RB] == 1 and trot_flag and stand_up_flag:
        stance_flag = True
        trot_flag = False
        gait_switch_flag = True
    # if A button was pressed, switch between auto mode and manual mode
    if data.buttons[A] == 1 and stand_up_flag:
        auto_mode_flag = not auto_mode_flag
        if auto_mode_flag:
            print("auto mode")
        else:
            print("manual mode")
    # if B button was pressed, execute the trajectory
    if data.buttons[B] == 1 and stand_up_flag:
        exec.data = True
        execute_publisher.publish(exec)
        print("execute trajectory")

    # X按钮进入或退出鸡头模式
    if data.buttons[X] == 1 and stand_up_flag:
        if not chicken_head_mode_flag:
            chicken_head_enter_flag = True
            print("进入鸡头模式")
        else:
            chicken_head_exit_flag = True
            print("退出鸡头模式")

    # RT按钮控制高度（仅在鸡头模式下有效）
    # RT和LT通常是模拟触发器，作为axes值而不是buttons值
    # 检测axes[LT]和axes[RT]的值，通常为-1到1之间，可能需要调整索引
    if chicken_head_mode_flag and len(data.axes) > 5:  # 确保有足够的轴
        # RT是axes[4]（若没有正确匹配，请根据实际手柄调整）
        rt_value = data.axes[4] if len(data.axes) > 5 else 0

        # 将RT的值（-1到1）直接映射到高度（0.3m到0.5m）
        # RT完全按下时为-1，映射到0.5m
        # RT未按下时为1，映射到0.3m
        new_height = 0.4 + (rt_value * 0.1)  # 简单线性映射公式

        # 确保高度在合理范围内
        new_height = max(0.3, min(0.5, new_height))

        # 仅在高度变化较大时更新
        if abs(chicken_head_pose.pose.position.z - new_height) > 0.005:
            chicken_head_pose.pose.position.z = new_height
            # 每0.05m打印一次高度信息
            if int(chicken_head_pose.pose.position.z * 20) != int(new_height * 20):
                print(f"高度调整为: {new_height:.2f}m (RT值: {rt_value:.2f})")

    # 设置常规控制信号
    tw.linear.x = 0.5 * data.axes[1]
    tw.linear.y = 0.5 * data.axes[0]
    tw.angular.z = data.axes[2]

    # 鸡头模式下的姿态控制
    if chicken_head_mode_flag:
        # 左摇杆控制yaw角度（范围±45度）
        # 在基准角度的基础上叠加摇杆输入，范围是±45度
        chicken_head_rpy[0] = chicken_head_base_yaw + math.radians(data.axes[2] * 45.0)

        # 右摇杆控制roll和pitch角度（范围±20度）
        # 注意：根据常见手柄配置，可能需要调整索引
        # 通常右摇杆是axes[3]和axes[4]，但有些手柄可能是[2]和[3]或其他
        chicken_head_rpy[1] = math.radians(data.axes[1] * 20.0)  # 左摇杆上下控制pitch
        chicken_head_rpy[2] = math.radians(data.axes[0] * -20.0)  # 左摇杆左右控制roll


def callback_state(data: mpc_observation):
    global x_obs, y_obs, z_obs, rpy_obs
    global sit_down_flag, stand_up_flag, init_flag
    x_obs = data.state.value[6]
    y_obs = data.state.value[7]
    z_obs = data.state.value[8]
    rpy_obs[0] = data.state.value[9]
    if init_flag:
        if z_obs < 0.1:
            sit_down_flag = True
            stand_up_flag = False
        elif z_obs > 0.4:
            sit_down_flag = False
            stand_up_flag = True
        init_flag = False
    # rpy_obs[1] = data.state.value[10]
    # rpy_obs[2] = data.state.value[11]
    # print(rpy_obs)


if __name__ == "__main__":
    try:
        rospy.init_node("lion_joy")
        joy_subscriber = rospy.Subscriber("joy", Joy, callback_joy)
        state_subscriber = rospy.Subscriber("legged_robot_mpc_observation", mpc_observation, callback_state)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # TODO get current pose

            if stand_up_enter_flag and not stand_up_flag and stance_flag:
                print("standing up")
                sit_down_pose.pose.position.z = sit_down_pose.pose.position.z + 0.05
                # sit_down_pose.pose.position.x = target_pose.pose.position.x
                # sit_down_pose.pose.position.y = target_pose.pose.position.y
                sit_down_pose.pose.position.x = x_obs
                sit_down_pose.pose.position.y = y_obs
                # rpy_obs to quaternion
                # restrict rpy_obs[0] to be in [0,2*pi]
                if rpy_obs[0] < 0:
                    rpy_obs[0] = rpy_obs[0] + 2 * math.pi
                elif rpy_obs[0] > 2 * math.pi:
                    rpy_obs[0] = rpy_obs[0] - 2 * math.pi
                quat = tf.transformations.quaternion_from_euler(rpy_obs[0], rpy_obs[1], rpy_obs[2])
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
                # stand_up_pose.pose.position.x = target_pose.pose.position.x
                # stand_up_pose.pose.position.y = target_pose.pose.position.y
                stand_up_pose.pose.position.x = x_obs
                stand_up_pose.pose.position.y = y_obs
                quat = tf.transformations.quaternion_from_euler(rpy_obs[0], rpy_obs[1], rpy_obs[2])
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

            # 处理鸡头模式进入和退出
            if chicken_head_enter_flag and stand_up_flag:
                print("进入鸡头模式中...")
                # 初始化鸡头姿态位置
                chicken_head_pose.pose.position.x = x_obs
                chicken_head_pose.pose.position.y = y_obs
                chicken_head_pose.pose.position.z = 0.5  # 默认高度
                # 设置初始方向为当前方向，并记录当前yaw作为基准值
                chicken_head_base_yaw = rpy_obs[0]  # 保存当前的yaw角度作为基准
                chicken_head_rpy = [chicken_head_base_yaw, 0.0, 0.0]  # 使用当前yaw角度，重置roll和pitch
                quat = tf.transformations.quaternion_from_euler(chicken_head_rpy[0], chicken_head_rpy[1], chicken_head_rpy[2])
                chicken_head_pose.pose.orientation.x = quat[0]
                chicken_head_pose.pose.orientation.y = quat[1]
                chicken_head_pose.pose.orientation.z = quat[2]
                chicken_head_pose.pose.orientation.w = quat[3]
                target_publisher.publish(chicken_head_pose)

                chicken_head_enter_flag = False
                chicken_head_mode_flag = True
                print(f"鸡头模式已启动，基准yaw角度: {math.degrees(chicken_head_base_yaw):.2f}°")

            elif chicken_head_exit_flag and chicken_head_mode_flag:
                print("退出鸡头模式中...")
                # 将头部姿态重置回零位，但保留yaw角度
                chicken_head_pose.pose.position.x = x_obs
                chicken_head_pose.pose.position.y = y_obs
                chicken_head_pose.pose.position.z = 0.5  # 重置高度为0.5m

                # 保留当前yaw角度，重置roll和pitch为零
                final_yaw = chicken_head_rpy[0]  # 保存最终的yaw角度
                quat = tf.transformations.quaternion_from_euler(final_yaw, 0.0, 0.0)
                chicken_head_pose.pose.orientation.x = quat[0]
                chicken_head_pose.pose.orientation.y = quat[1]
                chicken_head_pose.pose.orientation.z = quat[2]
                chicken_head_pose.pose.orientation.w = quat[3]
                target_publisher.publish(chicken_head_pose)

                # 更新target_rpy以保持一致性
                target_rpy[0] = final_yaw
                target_rpy[1] = 0.0
                target_rpy[2] = 0.0

                chicken_head_exit_flag = False
                chicken_head_mode_flag = False
                print(f"鸡头模式已退出，保留yaw角度: {math.degrees(final_yaw):.2f}°")

            if trot_flag and gait_switch_flag:
                print("trot mode")
                mode_publisher.publish(trot)
                gait_switch_flag = False
            elif stance_flag and gait_switch_flag:
                print("stance mode")
                mode_publisher.publish(stance)
                gait_switch_flag = False

            if trot_flag and not auto_mode_flag and not chicken_head_mode_flag:
                # vel_publisher.publish(tw)
                target_pose.pose.position.x = target_pose.pose.position.x + tw.linear.x * 0.1
                target_pose.pose.position.y = target_pose.pose.position.y + tw.linear.y * 0.1
                target_rpy[0] = target_rpy[0] + tw.angular.z * 0.1
                quat = tf.transformations.quaternion_from_euler(target_rpy[0], target_rpy[1], target_rpy[2])
                target_pose.pose.orientation.x = quat[0]
                target_pose.pose.orientation.y = quat[1]
                target_pose.pose.orientation.z = quat[2]
                target_pose.pose.orientation.w = quat[3]
                target_publisher.publish(target_pose)

            # 鸡头模式控制
            elif chicken_head_mode_flag:
                # 更新位姿
                quat = tf.transformations.quaternion_from_euler(chicken_head_rpy[0], chicken_head_rpy[1], chicken_head_rpy[2])
                chicken_head_pose.pose.orientation.x = quat[0]
                chicken_head_pose.pose.orientation.y = quat[1]
                chicken_head_pose.pose.orientation.z = quat[2]
                chicken_head_pose.pose.orientation.w = quat[3]
                target_publisher.publish(chicken_head_pose)

                # 每秒打印一次当前姿态信息（降低频率避免过多输出）
                if rospy.Time.now().to_sec() % 1 < 0.1:
                    yaw_deg = math.degrees(chicken_head_rpy[0])
                    roll_deg = math.degrees(chicken_head_rpy[1])
                    pitch_deg = math.degrees(chicken_head_rpy[2])
                    print(f"鸡头模式姿态: 高度={chicken_head_pose.pose.position.z:.2f}m, Yaw={yaw_deg:.1f}°, Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°")

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
