#ifndef ROSBAG2__ROSBAG2_HPP_
#define ROSBAG2__ROSBAG2_HPP_

#include <functional>
#include <string>

namespace rosbag2
{

class Rosbag2
{
public:
  // Support to playback ros1 bag files
  void play_ros1(const std::string & file_name);
};

}  // namespace rosbag2

#endif  // ROSBAG2__ROSBAG2_HPP_
