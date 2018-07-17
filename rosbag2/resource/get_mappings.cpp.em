// generated from rosbag2/resource/get_mappings.cpp.em

@###############################################
@#
@# Methods for determing mappings between
@# ROS 1 and ROS 2 interfaces
@#
@# EmPy template for generating get_mappings.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - mappings (list of ros1_bridge.Mapping)
@#    Mapping between messages as well as their fields
@###############################################
@
#include <map>
#include <string>

namespace rosbag2
{

bool
get_1to2_mapping(const std::string & ros1_type_name, std::string & ros2_type_name)
{
@[if not mappings]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[end if]@

@[for m in mappings]@
  if (ros1_type_name == "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)")
  {
    ros2_type_name = "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)";
    return true;
  }
@[end for]@

  return false;
}

}  // namespace rosbag2
