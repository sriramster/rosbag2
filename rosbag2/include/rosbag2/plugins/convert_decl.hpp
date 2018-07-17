#ifndef ROSBAG2__PLUGINS_CONVERT_DECL_HPP_
#define ROSBAG2__PLUGING_CONVERT_DECL_HPP_

namespace rosbag2
{

template<typename ROS1_T, typename ROS2_T>
void
convert_1_to_2(
  const ROS1_T & ros1_msg,
  ROS2_T & ros2_msg);

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__CONVERT_DECL_HPP_
