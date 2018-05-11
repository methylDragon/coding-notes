#include "PACKAGE_HEADERS/nodeClassInterface.hpp"

// STD
#include <string>

namespace node_class_interface {

RosNodeTemplate::RosNodeTemplate(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &RosNodeTemplate::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                               &RosNodeTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RosNodeTemplate::~RosNodeTemplate()
{
}

bool RosNodeTemplate::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void RosNodeTemplate::topicCallback(const std_msgs::Int32& message)
{
  algorithm_.addData(message.data);
}

bool RosNodeTemplate::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */
