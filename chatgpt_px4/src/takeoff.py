#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion  
from tf.transformations import quaternion_from_euler  
import rospy
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Path
# 假设这些是全局变量  

local_pos_pub = None  # 需要初始化  
rate = None  # 需要初始化  
current_pos = None  # 假设这是当前位置的PoseStamped消息  
global_flag = 0  # 全局标志，用于追踪command列表中的索引  
global_flag_reply = 0
command = []
pose_stamped_1 = PoseStamped()#初始化用到
current_positions = [] 


def callback(data, local_pos_pub, rate):  
    global command,global_flag_reply
    global_flag_reply = 1
    rospy.loginfo("Received: %s", data.data)
    # rospy.loginfo("6")
    try:
        command = json.loads(data.data)

        print(command)
        # handle_action(command, local_pos_pub, rate)
        
    except json.JSONDecodeError as e:
        rospy.logerr("Error decoding JSON: %s", e)
    except KeyError as e:
        rospy.logerr("Missing key in JSON data: %s", e)


pose_stamped = PoseStamped() 
# 检查是否需要移动并发布新的PoseStamped  
def check_and_move():  
    global global_flag, current_pos,pose_stamped,global_flag_reply
    rospy.loginfo("global_flag:%s",global_flag)
    rospy.loginfo("command_num:%s",len(command))
    rospy.loginfo("global_flag_reply:%s",global_flag_reply)
    
    # 假设current_pos已经被正确地设置了  
    if current_pos is None:  
        rospy.logerr("Current position is not set!")  
        return  
    # 假设我们有一个简单的距离检查（这里仅使用X轴作为示例）  
    # 注意：我们需要一个目标位置来比较，这里我们使用command列表中的位置  
    if global_flag < len(command) and global_flag_reply ==1: 
        # rospy.loginfo("1")
        target_position = command[global_flag]['position']  
        rospy.loginfo(target_position['x'])
        rospy.loginfo(current_pos.pose.position.x)

        if (abs(current_pos.pose.position.x - target_position['x']) < 0.25)and(abs(current_pos.pose.position.y - target_position['y']) < 0.25)and(abs(current_pos.pose.position.z- target_position['z']) < 0.25):  # 假设我们使用X轴  
            global_flag += 1  # 接近目标，移动到下一个命令
            rospy.loginfo("Moving robot to new position")    
        else:
            rospy.loginfo("继续当前指令")
        
         # 检查是否还有更多的命令  
        if global_flag < len(command):  
            if command[global_flag]['action'] == "move":  
                # 发布新的PoseStamped  
                # pose_stamped = PoseStamped()  
                pose_stamped.header.stamp = rospy.Time.now()  
                pose_stamped.header.frame_id = "map"  
                position = command[global_flag]['position']  
                pose_stamped.pose.position = Point(x=position['x'], y=position['y'], z=position['z'])  
                q = quaternion_from_euler(0, 0, 0)  
                pose_stamped.pose.orientation = Quaternion(*q)  
                local_pos_pub.publish(pose_stamped)  
                
            else:  
                rospy.loginfo("Non-moving command received: %s", command[global_flag]['action'])  
        else:  
            rospy.loginfo("Reached the end of command list.")  
    else:
        rospy.loginfo("路径点执行完毕,flag置零") 
        global_flag_reply = 0
        global_flag = 0 
        # rospy.loginfo(pose_stamped)  
        local_pos_pub.publish(pose_stamped)  


    rate.sleep()  


current_state = State()
# 每次订阅到/mavros/state的消息就调用state_cb函数，将订阅到的消息赋值给全局变量current_state
def state_cb(msg):
    global current_state
    current_state = msg

current_pos= PoseStamped()
# 每次订阅到/mavros/local_position/pose的消息就调用pos_cb函数，将订阅到的消息赋值给全局变量current_pos
def  pos_cb(msg):
    global current_pos,current_positions
    current_pos = msg
    current_positions.append(msg) 
    # 创建一个新的Path消息  
    path = Path()  
    path.header.frame_id = "map"  # 或者设置为与PoseStamped相同的frame_id  
    path.header.stamp = rospy.Time.now()  # 设置当前时间戳  
    # 将current_positions列表中的PoseStamped消息添加到Path消息的poses中  
    for pose in current_positions:  
        path_pose = PoseStamped()  
        path_pose.pose = pose.pose  
        path.poses.append(path_pose)  
    # path_pose = PoseStamped()  
    # path_pose.pose = current_pos.pose  
    path.poses.append(path_pose)  
    # 发布Path消息  
    path_pub.publish(path) 




if __name__ == "__main__":
    rospy.init_node("multi_uav")
    rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos_cb,queue_size=10)
    armServer = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    setModeServer = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    path_pub = rospy.Publisher('path', Path, queue_size=10)  
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)  
    rate = rospy.Rate(20)  
    rospy.Subscriber("gpt_reply_to_user", String, lambda data: callback(data, local_pos_pub, rate)) 
    

    # pose_start = PoseStamped()  

    # pose_start.pose.position = Point(0,0,0)  # 设置位置  
    # # 设置姿态（这里使用欧拉角转换为四元数）  
    # # 例如，设置偏航角（yaw）为90度（即π/2弧度），俯仰和翻滚为0  
    # q = quaternion_from_euler(0, 0, 0)  # 假设yaw角不改变
    # pose_start.pose.orientation = Quaternion(*q)  
    # # 发布消息  
    # rospy.loginfo(pose_start)

    # count = 100
    # while not rospy.is_shutdown() and count > 0:  
    #     local_pos_pub.publish(pose_start)  
    #     # 减少计数器  
    #     count -= 1 
        
    

    # global_flag = 0
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" :
            setModeServer(custom_mode='OFFBOARD')
            local_pos_pub.publish(pose_stamped_1)   
            print("Offboard enabled")
        else:
            if not current_state.armed:
                armServer(True) 
                print("Vehicle armed")

        check_and_move()
        # rospy.loginfo("5")
        # local_pos_pub.publish(pose_stamped)    
        # rospy.loginfo(pose_stamped)
        rate.sleep() 

