#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "param_info_node");
    ros::NodeHandle nh;

    nh.setParam("/info_param", "methylDragon");

    ros::Rate loopRate(10);

    while (ros::ok())
    {
        std::string info;

        nh.getParam("/info_param", info);

        ROS_INFO_STREAM("info_param value: " + info);
        ROS_INFO_STREAM("To test it out some more, use: rosparam set /info_param <whatever_you_want>\n");

        loopRate.sleep();
    }

    return 0;
}
