#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import httpx
from openai import OpenAI

# 初始化OpenAI客户端
client = OpenAI(
    base_url="你的代理地址", #代理地址修改这里
    api_key="your key", #你的代理地址提供给你的Api_key
    http_client=httpx.Client(
        base_url="你的代理地址",
        follow_redirects=True,
    ),
)


def chat_with_gpt(gpt_reply_pub):  
    # rospy.init_node('chatgpt_ros_node', anonymous=True)

    # # 订阅用户消息，用户输入：rostopic pub /user_to_gpt std_msgs/String "'Hello'"
    # rospy.Subscriber("user_to_gpt", String, chat_with_gpt)

    # # 创建发布者，用于发布GPT的回复
    # gpt_reply_pub = rospy.Publisher("gpt_reply_to_user", String, queue_size=10)

    # 初始化对话，通常包含一个空的对话历史  
    # 初始化变量来存储对话历史
    conversation_history = []  
    
    while True:  
        try:  
            # 在终端中等待用户输入  
            user_input = input("\nType your question or 'exit' to quit: ")  
            print("User:",user_input) 

            if user_input.lower() == 'exit':  
                break  # 如果用户输入'exit'，则退出循环  
  
            # 构造GPT请求的参数 
            priori_knowledge =  """你是一个专业的PX4无人机路径规划器、你可以根据我的需求规划无人机三维空间中的飞行轨迹。
            请给我一系列以下JSON格式的回复: 
            [{\"action\": \"move\",\"position\": {\"x\": [desire position value in x axis],\"y\": [desire position value in y axis],\"z\": [desire position value in z axis]}},{\"action\": \"move\",\"position\": {\"x\": [desire position value in x axis],\"y\": [desire position value in y axis],\"z\": [desire position value in z axis]}}……].
            The 'position' value controls the quadcopter's desire position in 3-axis.
            注意、z轴方向须等于2米。
            例如，当我要求你规划一个矩形轨迹时，你可以给出类似回复如下：
            [{\"action\": \"move\",\"position\": {\"x\": 0,\"y\": 0,\"z\": 2}},
            {\"action\": \"move\",\"position\": {\"x\": 2,\"y\": 0,\"z\": 2}},
            {\"action\": \"move\",\"position\": {\"x\": 4,\"y\": 0,\"z\": 2}},
            {\"action\": \"move\",\"position\": {\"x\": 4,\"y\": 2,\"z\": 2}},
            {\"action\": \"move\",\"position\": {\"x\": 4,\"y\": 4,\"z\": 2}},
            {\"action\": \"move\",\"position\": {\"x\": 2,\"y\": 4,\"z\": 2}},
            {\"action\": \"move\",\"position\": {\"x\": 0,\"y\": 4,\"z\": 2}}，
            {\"action\": \"move\",\"position\": {\"x\": 0,\"y\": 2,\"z\": 2}},
            {\"action\": \"move\",\"position\": {\"x\": 0,\"y\": 0,\"z\": 2}}]
            注意、每次规划完毕之后默认无人机的位置停留在上一次的结束位置，这意味着下一次你规划的起点就是上一次你规划的终点"""
            messages = [   
                {"role": "user", "content": priori_knowledge},  
            ]  
            
            # print("A")     
            # 如果存在先前的conversation，则使用它  
            if conversation_history:  
                conversation_history.append({"role": "assistant", "content": user_input}) 
                response = client.chat.completions.create(
                model="gpt-3.5-turbo",  
                messages = conversation_history
                ) 
            else:
                conversation_history.append({"role": "assistant", "content": user_input}) 
                response = client.chat.completions.create(
                model="gpt-3.5-turbo",  
                messages = messages
                ) 



            # 获取GPT的回复  
            gpt_reply = response.choices[0].message.content

            # 打印GPT的回复到终端  
            print("GPT Reply:", gpt_reply)  
            conversation_history.append({"role": "assistant", "content": gpt_reply}) 
            # print("conversation_history",conversation_history) 

            gpt_reply_pub.publish(gpt_reply)

  
        except Exception as e:  
            print(f"An error occurred: {e}")  

if __name__ == '__main__':
    try:
        rospy.init_node('chatgpt_ros_node', anonymous=True)

        # 订阅用户消息，用户输入：rostopic pub /user_to_gpt std_msgs/String "'Hello'"
        # rospy.Subscriber("user_to_gpt", String, chat_with_gpt)

        # 创建发布者，用于发布GPT的回复
        gpt_reply_pub = rospy.Publisher("gpt_reply_to_user", String, queue_size=10)

        chat_with_gpt(gpt_reply_pub)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
