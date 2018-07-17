// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosbag2/plugins/convert_builtin_interfaces.hpp"

namespace rosbag2
{

template<>
void
convert_1_to_2(
  const ros::Duration & ros1_type,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  ros2_msg.sec = ros1_type.sec;
  ros2_msg.nanosec = ros1_type.nsec;
}

template<>
void
convert_1_to_2(
  const ros::Time & ros1_type,
  builtin_interfaces::msg::Time & ros2_msg)
{
  ros2_msg.sec = ros1_type.sec;
  ros2_msg.nanosec = ros1_type.nsec;
}

}  // namespace ros1_bridge
