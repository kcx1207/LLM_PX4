#include <ros/ros.h>  
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h>  
#include <message_filters/synchronizer.h>  
  
class SetpointRelayNode {  
public:  
    SetpointRelayNode() : nh_("~") {  
        // 订阅/iris_0/mavros/setpoint_raw/local  
        setpoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("iris_0/mavros/setpoint_raw/local", 10, &SetpointRelayNode::setpointCallback, this);  
  
        // 发布到/mavros/setpoint_raw/local  
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("AAAmavros/setpoint_raw/local", 10);  
    }  
  
    void setpointCallback(const geometry_msgs::PoseStampedConstPtr& msg) {  
        // 直接发布接收到的消息  
        setpoint_pub_.publish(*msg);  
    }  
  
private:  
    ros::NodeHandle nh_;  
    ros::Subscriber setpoint_sub_;  
    ros::Publisher setpoint_pub_;  
};  
  
int main(int argc, char** argv) {  
    ros::init(argc, argv, "setpoint_relay_node");  
  
    SetpointRelayNode relay_node;  
  
    ros::spin();  
  
    return 0;  
}