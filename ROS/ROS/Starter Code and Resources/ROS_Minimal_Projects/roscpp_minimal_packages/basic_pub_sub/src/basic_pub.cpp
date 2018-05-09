#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
    // New node called talker
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    // Publish onto the "chatter" topic with a queue size of 1
    ros::Publisher chatter_publisher = nh.advertise<std_msgs::String>("chatter", 1);

    ros::Rate loopRate(10);

    unsigned int count = 0;

    while (ros::ok())
    {
        std_msgs::String message; // Make a new message object
        std::ostringstream string_count;
        string_count << count;

        message.data = "Rawr " + string_count.str(); // Write to it
        ROS_INFO_STREAM(message.data);

        chatter_publisher.publish(message);

        ros::spinOnce();
        loopRate.sleep();
        count++;
    }

    return 0;
}
