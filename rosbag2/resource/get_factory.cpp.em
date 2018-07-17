// generated from visteon_rosbag/resource/get_factory.cpp.em

@###############################################
@#
@# Factory for creating publisher / subscribers
@# based on message names
@#
@# EmPy template for generating get_factory.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_names (list of str)
@#    ROS 2 package names
@###############################################
@
@{
from rosbag2 import camel_case_to_lower_case_underscore
}@
#include "rosbag2/plugins/factory.hpp"
#include "rosbag2/plugins/builtin_interfaces_factories.hpp"

@[for ros2_package_name in sorted(ros2_package_names)]@
#include "@(ros2_package_name)_factories.hpp"
@[end for]@

namespace rosbag2
{

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros1_type_name, const std::string & ros2_type_name)
{
@[if not ros2_package_names]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[else]@
  std::shared_ptr<FactoryInterface> factory;
@[end if]@
  factory = get_factory_builtin_interfaces(ros1_type_name, ros2_type_name);
  if (factory) {
    return factory;
  }
@[for ros2_package_name in sorted(ros2_package_names)]@
  factory = get_factory_@(ros2_package_name)(ros1_type_name, ros2_type_name);
  if (factory) {
    return factory;
  }
@[end for]@
  return nullptr;
}

}  // namespace rosbag2
