#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion  
from tf.transformations import quaternion_from_euler  

def callback(data, local_pos_pub, rate):  
    rospy.loginfo("Received: %s", data.data)
    
    try:
        command = json.loads(data.data)

        # 处理命令
        handle_action(command, local_pos_pub, rate)
        print(command)
    except json.JSONDecodeError as e:
        rospy.logerr("Error decoding JSON: %s", e)
    except KeyError as e:
        rospy.logerr("Missing key in JSON data: %s", e)

def handle_action(command, local_pos_pub, rate): 

    if command["action"] == "move":
        rospy.loginfo("Moving robot")
        # 使用新的JSON格式设置位置
        while not rospy.is_shutdown():
            # 创建一个新的PoseStamped消息  
            pose_stamped = PoseStamped()  
            pose_stamped.header.stamp = rospy.Time.now()  # 设置时间戳为当前时间  
            pose_stamped.header.frame_id = "map"  # 设置参考坐标系  

            # 设置pose的位置和姿态  
            pose_stamped.pose.position = Point(x=command["position"]["x"], y=command["position"]["y"], z=command["position"]["z"])  # 设置位置  

            # 设置姿态（这里使用欧拉角转换为四元数）  
            # 例如，设置偏航角（yaw）为90度（即π/2弧度），俯仰和翻滚为0  
            q = quaternion_from_euler(0, 0, 0)  # 假设yaw角不改变
            pose_stamped.pose.orientation = Quaternion(*q)  

            # 发布消息  
            rospy.loginfo(pose_stamped)
            local_pos_pub.publish(pose_stamped) 
            rate.sleep()  

    else:
        rospy.logerr("Unknown action: %s", command.get("action"))


def local_pos_publisher():
    # 初始化节点  
    rospy.init_node('local_pos_publisher', anonymous=True)  
    # 创建一个Publisher，发布geometry_msgs/PoseStamped类型的消息到mavros/setpoint_position/local主题  
    # 队列大小设置为10  
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)  
    # 设置发布频率  
    rate = rospy.Rate(10)  # 10Hz  
    rospy.Subscriber("gpt_reply_to_user", String, lambda data: callback(data, local_pos_pub, rate))
    rospy.spin()


if __name__ == '__main__':
    try:  
        local_pos_publisher()  
    except rospy.ROSInterruptException:  
        pass 

