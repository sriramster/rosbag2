#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// include builtin interfaces
#include "rosbag2/plugins/builtin_interfaces_factories.hpp"
#include "rosbag2/plugins/convert_builtin_interfaces.hpp"

namespace rosbag2
{

std::shared_ptr<FactoryInterface>
get_factory_builtin_interfaces(
  const std::string & ros1_type_name,
  const std::string & ros2_type_name)
{
  // mapping from string to specialized template
  if (
    (ros1_type_name == "std_msgs/Duration" || ros1_type_name == "") &&
    ros2_type_name == "builtin_interfaces/Duration")
  {
    return std::make_shared<
      Factory<
        std_msgs::Duration,
        builtin_interfaces::msg::Duration
      >
    >("std_msgs/Duration", ros2_type_name);
  }
  if (
    (ros1_type_name == "std_msgs/Time" || ros1_type_name == "") &&
    ros2_type_name == "builtin_interfaces/Time")
  {
    return std::make_shared<
      Factory<
        std_msgs::Time,
        builtin_interfaces::msg::Time
      >
    >("std_msgs/Time", ros2_type_name);
  }

  return std::shared_ptr<FactoryInterface>();
}

// conversion functions for available interfaces
template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::convert_1_to_2(
  const std_msgs::Duration & ros1_msg,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  rosbag2::convert_1_to_2(ros1_msg.data, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::convert_1_to_2(
  const std_msgs::Time & ros1_msg,
  builtin_interfaces::msg::Time & ros2_msg)
{
  rosbag2::convert_1_to_2(ros1_msg.data, ros2_msg);
}
}  // namespace ros1_bridge
