#!/usr/bin/env python3  

  

import rospy  

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion  

from tf.transformations import quaternion_from_euler  

  

def local_pos_publisher():  

    # 初始化节点  

    rospy.init_node('local_pos_publisher', anonymous=True)  

  

    # 创建一个Publisher，发布geometry_msgs/PoseStamped类型的消息到mavros/setpoint_position/local主题  

    # 队列大小设置为10  

    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)  

  

    # 设置发布频率  

    rate = rospy.Rate(10)  # 10Hz  

  

    while not rospy.is_shutdown():  

        # 创建一个新的PoseStamped消息  

        pose_stamped = PoseStamped()  

        pose_stamped.header.stamp = rospy.Time.now()  # 设置时间戳为当前时间  

        pose_stamped.header.frame_id = "map"  # 设置参考坐标系  

  

        # 设置pose的位置和姿态  

        pose_stamped.pose.position = Point(x=1.0, y=2.0, z=3.0)  # 设置位置  

  

        # 设置姿态（这里使用欧拉角转换为四元数）  

        # 例如，设置偏航角（yaw）为90度（即π/2弧度），俯仰和翻滚为0  

        q = quaternion_from_euler(0, 0, rospy.get_time() / 2.0)  # 假设我们随时间改变偏航角  

        pose_stamped.pose.orientation = Quaternion(*q)  

  

        # 发布消息  

        local_pos_pub.publish(pose_stamped)  

  

        # 按照设定的频率休眠  

        rate.sleep()  

  

if __name__ == '__main__':  

    try:  

        local_pos_publisher()  

    except rospy.ROSInterruptException:  

        pass
