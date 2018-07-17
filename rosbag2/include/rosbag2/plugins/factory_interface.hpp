#ifndef  ROSBAG2__PLUGINS_FACTORY_INTERFACE_HPP_
#define  ROSBAG2__PLUGINS_FACTORY_INTERFACE_HPP_

#include <string>

#include <rosbag/view.h>
#include <rosbag/bag.h>

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

namespace rosbag2
{
class FactoryInterface
{
public:
  virtual
  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  void
  publish_ros1_msg(
    rclcpp::PublisherBase::SharedPtr pub,
    const rosbag::MessageInstance & m) = 0;
};

}  // namespace rosbag2

#endif  // ROSBAG2__PLUGINS_FACTORY_INTERFACE_HPP_
