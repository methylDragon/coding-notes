#include <ros/ros.h> // Include the ROS main header file

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rawrer", ros::init_options::AnonymousName);
    // Initialise a node named rawrer, make it anonymous
    // Check the rospy hello_world for what anonymous means

    ros::NodeHandle node_handle; // This HANDLES all communications with ROS
    // Topics, services, parameters, etc

    ros::Rate loopRate(10); // Hz

    // Let's just have a nice incrementer here
    unsigned int count = 0;

    // As long as ROS is running, keep running
    while (ros::ok())
    { // ros::ok() checks for the status of ROS
        ROS_INFO_STREAM("Rawr " << count); // Log it!
        // Notice how we aren't using cout here! Use ROS_INFO and ROS_INFO_STREAM!

        ros::spinOnce(); // Calls all callbacks waiting to be called-back
        loopRate.sleep(); // Sleep according to the loopRate
        count++; // Aaaand increment our counter
    }

    return 0;
}
