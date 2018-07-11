#include <ros/ros.h>
#include <msg_example_msgs/My_msg.h>

msg_example_msgs::My_msg msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msg_example_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_publisher = nh.advertise<msg_example_msgs::My_msg>("msg_example", 1);

    ros::Rate loopRate(10);

    unsigned int count = 0;

    while (ros::ok())
    {
        msg_example_msgs::My_msg msg; // Make a new message object
        msg.header.stamp = ros::Time::now();
        
        msg.name = "methylDragon";
        msg.dragon_rating = 10;
        
        ROS_INFO_STREAM("       .     .\n"
                        "                                    .  |\\-^-/|  .\n"
                        "FIND MY OUTPUT ON /msg_example !   /| } O.=.O { |\\ \n");

        chatter_publisher.publish(msg);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
