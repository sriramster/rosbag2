#ifndef ROSBAG2_PLUGINS_FACTORY_HPP_
#define ROSBAG2_PLUGINS_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rmw/rmw.h"

#include "ros/message.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "rcutils/logging_macros.h"

#include "factory_interface.hpp"

namespace rosbag2
{

template<typename ROS1_T, typename ROS2_T>
class Factory : public FactoryInterface
{
 public:
  Factory(
      const std::string & ros1_type_name,
      const std::string & ros2_type_name)
      : ros1_type_name_(ros1_type_name),
        ros2_type_name_(ros2_type_name)
  {}

  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
      rclcpp::Node::SharedPtr node,
      const std::string & topic_name,
      size_t queue_size)
  {
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = queue_size;
    std::cout<<"Creating msgs publishers"<<std::endl;
    return node->create_publisher<ROS2_T>(topic_name, custom_qos_profile);
  }

  void
  publish_ros1_msg(
      rclcpp::PublisherBase::SharedPtr pub,
      const rosbag::MessageInstance & m)
  {
    std::cout<<"Publish msgs"<<std::endl;
    typename rclcpp::Publisher<ROS2_T>::SharedPtr typed_ros2_pub;
    typed_ros2_pub =
        std::dynamic_pointer_cast<typename rclcpp::Publisher<ROS2_T>>(pub);

    auto ros1_msg = boost::shared_ptr<ROS1_T const>();
    ros1_msg = m.instantiate<ROS1_T>();
    auto ros2_msg = std::make_shared<ROS2_T>();

    if (ros1_msg != NULL) {
      std::cout<<"Trying to convert"<<std::endl;
      convert_1_to_2(*ros1_msg, *ros2_msg);
      typed_ros2_pub->publish(ros2_msg);
    }
  }
  
 public:
  static
  void
  convert_1_to_2(
      const ROS1_T & ros1_msg,
      ROS2_T & ros2_msg);

  std::string ros1_type_name_;
  std::string ros2_type_name_;

};
}  // namespace rosbag2

#endif //ROSBAG2_PLUGINS_ROS1READER_HPP_
