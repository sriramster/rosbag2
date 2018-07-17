#ifndef ROSBAG2_PLUGINS_ROS1_PLAYER_H_
#define ROSBAG2_PLUGINS_ROS1_PLAYER_H_

#include <map>

#include <rosbag/message_instance.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <rclcpp/rclcpp.hpp>

#include <boost/foreach.hpp>

#include "factory.hpp"
#include "factory_interface.hpp"

#define foreach BOOST_FOREACH

using namespace rosbag;

namespace rosbag2 {

std::map<std::string, rclcpp::PublisherBase::SharedPtr> t_pub_map;

bool
get_1to2_mapping(
  const std::string & ros1_type_name,
  std::string & ros2_type_name);

std::shared_ptr<FactoryInterface>
get_factory(
  const std::string & ros1_type_name,
  const std::string & ros2_type_name);

bool
find(const std::string & topic_name)
{
  std::map<std::string, rclcpp::PublisherBase::SharedPtr>::iterator it;

  it = t_pub_map.find(topic_name);

  if (it == t_pub_map.end()) {
    return false;
  }

  return true;
}

std::map<std::string, rclcpp::PublisherBase::SharedPtr>::iterator
find_it(const std::string & topic_name)
{
  std::map<std::string, rclcpp::PublisherBase::SharedPtr>::iterator it;

  it = t_pub_map.find(topic_name);

  if (it != t_pub_map.end()) {
    return it;
  }
}

bool
create_ros2_publishers(const std::string & type_name, const std::string & topic_name, rclcpp::Node::SharedPtr node, size_t queue_sz)
{
  auto factory = get_factory(type_name, type_name);
  if(factory != nullptr) {
    auto pub_ = factory->create_ros2_publisher(node, topic_name, queue_sz);
    t_pub_map[topic_name] = pub_;
  }
  else {
    return false;
  }

  return true;
}

bool
publish_ros1_msg(const rosbag::MessageInstance & m, const std::string & topic_name, const std::string & type_name)
{

  std::map<std::string, rclcpp::PublisherBase::SharedPtr>::iterator it;

  // TODO:[sriram] optimize this below
  if (find(topic_name)) {
    it = find_it(topic_name);
  }
  else {
    std::cout<<"Iterator not found = "<<type_name<<std::endl;
    return true;
  }

  auto factory = get_factory(type_name, type_name);
  if (factory != nullptr) {
    factory->publish_ros1_msg(it->second, m);
    std::cout<<"Published"<<std::endl;
    return true;
  }
  return false;
}

bool
start_ros1bag_player(const std::string & file_name)
{
  boost::shared_ptr<Bag> bag(boost::make_shared<Bag>());

  if (!file_name.empty()) {
    try {
      bag->open(file_name);
    }
    catch (BagUnindexedException ex) {
      std::cerr << "Bag file " << file_name << " is unindexed.  Run rosbag reindex." << std::endl;
      return false;
    }
  }

  View view;
  view.addQuery(*bag);

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  auto node_handle_ = std::make_shared<rclcpp::Node>("r2ros1player");
  
  foreach(const ConnectionInfo* c, view.getConnections())
  {
    ros::M_string::const_iterator header_iter = c->header->find("callerid");
    std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

    std::string callerid_topic = callerid + c->topic;
    std::string type = c->datatype;
    size_t queue_sz = 7;
    std::cout<<"Type" <<type<<std::endl;
    bool res = create_ros2_publishers(type, callerid_topic, node_handle_, queue_sz);
    if (!res)
      std::cout<<"Unable to create publisher from type"<<type<<std::endl;
  }

  while (true) {
    foreach(MessageInstance m, view) {
      if (!rclcpp::ok())
        break;

      std::string const& topic   = m.getTopic();
      std::string callerid       = m.getCallerId();
    
      std::string callerid_topic = callerid + topic;

      if (!publish_ros1_msg(m, callerid_topic, m.getDataType()))
        throw std::runtime_error("Could'nt publish");
    }

    if (!rclcpp::ok()) {
      std::cout << std::endl;
      break;
    }
  }
}

}

#endif //ROSBAG2_PLUGINS_ROS1_PLAYER_H_
