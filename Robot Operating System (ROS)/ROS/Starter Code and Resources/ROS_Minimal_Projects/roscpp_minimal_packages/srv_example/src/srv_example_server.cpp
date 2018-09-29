#include <ros/ros.h>
#include <srv_example/AddTwoInts.h>

// Callback function
bool add(srv_example::AddTwoInts::Request &request,
         srv_example::AddTwoInts::Response &response)
{
  response.Sum = request.A + request.B;
  
  ROS_INFO("request: x=%ld, y=%ld", (long int)request.A, (long int)request.B);
  
  ROS_INFO(" sending back response: [%ld]", (long int)response.Sum);
  
  return true;
}

int main(int argc, char **argv)
{
  // Start the node
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle nh;
  
  // Advertise the service
  ros::ServiceServer service = nh.advertiseService("add_two_ints", add);

  ROS_INFO("Ready to add two ints.");

  // Spin up
  ros::spin();

  return 0;
}
