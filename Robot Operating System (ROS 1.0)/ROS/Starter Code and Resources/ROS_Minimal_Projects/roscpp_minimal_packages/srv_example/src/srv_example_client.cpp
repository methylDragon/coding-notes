#include <ros/ros.h>
#include <srv_example/AddTwoInts.h>
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_two_ints_client");

  if (argc != 3) {
    ROS_INFO("Usage: %s [x y]", argv[0]);
    return 1;
}

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<srv_example::AddTwoInts>("add_two_ints");

srv_example::AddTwoInts service;
service.request.A = atoi(argv[1]);
service.request.B = atoi(argv[2]);

// Service callback function
// client.call(service) calls the client!
if (client.call(service)) {
  ROS_INFO("Sum: %ld", (long int)service.response.Sum);
}
else {
  ROS_ERROR("Failed to call service add_two_ints");
  return 1;
}

  return 0;
} 
