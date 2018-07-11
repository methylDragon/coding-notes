#include "ros/ros.h"
#include "std_msgs/String.h"

void chatter_callback(const std_msgs::String& msg)
{
	ROS_INFO("I heard: [%s]", msg.data.c_str());
}

int main(int argc, char **argv)
{
    // New node called listener
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    
    // Subscribe to chatter with a queue size of 10 and callback
    ros::Subscriber chatter_subscriber = nh.subscribe("chatter", 10, chatter_callback);
    ros::spin(); // Keep running until the node is killed
    
    return 0;
}
