#include "rclcpp/rclcpp.hpp"

#include "rosbag2/plugins/factory.hpp"
#include "rosbag2/plugins/factory_interface.hpp"
#include "rosbag2/plugins/ros1_player.hpp"

int main(int argc, const char ** argv)
{
  rclcpp::init(argc, argv);

  std::string file_name = argv[1];
  std::cout<<"Opening rosbag file" << file_name<<std::endl;

  rosbag2::start_ros1bag_player(file_name);

  rclcpp::shutdown();

  return 0;
}
