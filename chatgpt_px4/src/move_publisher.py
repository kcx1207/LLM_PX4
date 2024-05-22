#!/usr/bin/env python3
  
import rospy  
import json  
from std_msgs.msg import String  
  
def move_publisher():  
    # 初始化节点  
    rospy.init_node('move_publisher_node', anonymous=True)  
  
    # 创建一个Publisher，发布到/gpt_reply_to_user，消息类型为String，队列大小10  
    pub = rospy.Publisher('gpt_reply_to_user', String, queue_size=10)  
  
    # 定义移动指令列表  
    move_commands = [  
        {"action": "move", "position": {"x": 10, "y": 5, "z": 2}},  
        {"action": "move", "position": {"x": 15, "y": 7, "z": 2}},  
        {"action": "move", "position": {"x": 20, "y": 3, "z": 2}},
        {"action": "move", "position": {"x": 0, "y": 0, "z": 2}}  
    ]  
    # 创建了一个包含多个字典的列表move_commands，并使用json.dumps()函数将其序列化为一个JSON格式的字符串json_string，用于测试
    json_string = json.dumps(move_commands)  
    rate = rospy.Rate(0.05) 
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing move commands: %s", json_string)  
        pub.publish(json_string)   
        rate.sleep() 
  
if __name__ == '__main__':  
    try:  
        move_publisher()  
    except rospy.ROSInterruptException:  
        pass
