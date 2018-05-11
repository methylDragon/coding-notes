#pragma once

#include "PACKAGE_HEADERS/algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

namespace node_class_interface {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosNodeTemplate
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RosNodeTemplate(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RosNodeTemplate();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const std_msgs::Int32& message);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  Algorithm algorithm_;
};

} /* namespace */
