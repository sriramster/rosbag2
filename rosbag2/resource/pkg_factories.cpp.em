// generated from visteon_rosbag/resource/pkg_factories.cpp.em

@###############################################
@#
@# Factory template specializations based on
@# message types of a single ROS 2 package
@#
@# EmPy template for generating <pkgname>_factories.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_name (str)
@#    The ROS 2 package name of this file
@#  - mappings (list of visteon_rosbag.Mapping)
@#    Mapping between messages as well as their fields
@###############################################
@
@{
from rosbag2 import camel_case_to_lower_case_underscore
}@
#include "rclcpp/rclcpp.hpp"

#include "@(ros2_package_name)_factories.hpp"

// include builtin interfaces
#include <rosbag2/plugins/convert_builtin_interfaces.hpp>

namespace rosbag2
{

std::shared_ptr<FactoryInterface>
get_factory_@(ros2_package_name)(const std::string & ros1_type_name, const std::string & ros2_type_name)
{
@[if not mappings]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[end if]@
  // mapping from string to specialized template
@[for m in mappings]@
  if (
    (ros1_type_name == "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)" ||
     ros1_type_name == "") &&
    ros2_type_name == "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)")
  {
    return std::make_shared<
      Factory<
        @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name),
        @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name)
      >
    >("@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)", ros2_type_name);
  }
@[end for]@
  return std::shared_ptr<FactoryInterface>();
}

// conversion functions for available interfaces
@[for m in mappings]@

template<>
void
Factory<
  @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name),
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name)
>::convert_1_to_2(
  const @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name) & ros1_msg,
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name) & ros2_msg)
{
@[  if not m.fields_1_to_2]@
  (void)ros1_msg;
  (void)ros2_msg;
@[  end if]@
@[  for ros1_field, ros2_field in m.fields_1_to_2.items()]@
@[    if not ros2_field.type.is_array]@
  // convert non-array field
@[      if not ros2_field.type.pkg_name]@
  // convert primitive field
  ros2_msg.@(ros2_field.name) = ros1_msg.@(ros1_field.name);
@[      elif ros2_field.type.pkg_name == 'builtin_interfaces']@
  // convert builtin field
  rosbag2::convert_1_to_2(ros1_msg.@(ros1_field.name), ros2_msg.@(ros2_field.name));
@[      else]@
  // convert sub message field
  Factory<
    @(ros1_field.pkg_name)::@(ros1_field.msg_name),
    @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
  >::convert_1_to_2(
    ros1_msg.@(ros1_field.name), ros2_msg.@(ros2_field.name));
@[      end if]@
@[    else]@
  // convert array field
@[      if not ros2_field.type.array_size or ros2_field.type.is_upper_bound]@
  // ensure array size
@[        if ros2_field.type.is_upper_bound]@
  // check boundary
  assert(ros1_msg.@(ros1_field.name).size() <= @(ros2_field.type.array_size));
@[        end if]@
  // dynamic arrays must be resized
  ros2_msg.@(ros2_field.name).resize(ros1_msg.@(ros1_field.name).size());
@[      end if]@
@[      if not ros2_field.type.pkg_name]@
  // convert primitive array elements
  std::copy(
    ros1_msg.@(ros1_field.name).begin(),
    ros1_msg.@(ros1_field.name).end(),
    ros2_msg.@(ros2_field.name).begin());
@[      else]@
  // copy element wise since the type is different
  {
    auto ros1_it = ros1_msg.@(ros1_field.name).begin();
    auto ros2_it = ros2_msg.@(ros2_field.name).begin();
    for (
      ;
      ros1_it != ros1_msg.@(ros1_field.name).end() &&
      ros2_it != ros2_msg.@(ros2_field.name).end();
      ++ros1_it, ++ros2_it
    )
    {
      // convert sub message element
@[        if ros2_field.type.pkg_name == 'builtin_interfaces']@
	        rosbag2::convert_1_to_2(*ros1_it, *ros2_it);
@[        else]@
      Factory<
        @(ros1_field.pkg_name)::@(ros1_field.msg_name),
        @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
      >::convert_1_to_2(
        *ros1_it, *ros2_it);
@[        end if]@
    }
  }
@[      end if]@
@[    end if]@
@[  end for]@
}

@[end for]@

}  // namespace rosbag2
