#include <ros/ros.h>
#include "PACKAGE_HEADERS/nodeClassInterface.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Node");
  ros::NodeHandle nodeHandle("~");

  // Construct the node 
  // (assigning to variable Node_object, but you can call it anything you want)
  node_class_interface::RosNodeTemplate Node_object(nodeHandle);

  // Run it
  ros::spin();

  return 0;
}
