#ifndef ROSBAG2_PLUGINS_ROS1_PLAYER_H_
#define ROSBAG2_PLUGINS_ROS1_PLAYER_H_

#include <map>

#include <rosbag/message_instance.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

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
    it = t_pub_map.find(topic_name);

    if (it == t_pub_map.end()) {
      return false;
    }
  }

  auto factory = get_factory(type_name, type_name);
  if (factory != nullptr) {
    factory->publish_ros1_msg(it->second, m);
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

  auto node_handle_ = std::make_shared<rclcpp::Node>("r2ros1player");
  auto type_missed = false;
  std::map<std::string, std::string> type_missed_map;
  
  foreach(const ConnectionInfo* c, view.getConnections())
  {
    ros::M_string::const_iterator header_iter = c->header->find("callerid");
    std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

    std::string callerid_topic = callerid + c->topic;
    std::string type = c->datatype;
    size_t queue_sz = 7;
    bool res = create_ros2_publishers(type, callerid_topic, node_handle_, queue_sz);
    if (!res) {
      type_missed = true;
      type_missed_map[callerid_topic] = type;
    }
  }

  if (type_missed) {
    // RCUTILS_LOG_DEBUG("Unable to create publisher for sometypes since support is unavailable please check mapping");

    for (std::map<std::string, std::string>::iterator it = type_missed_map.begin(); it != type_missed_map.end(); it++) {
      std::cout<<it->first << " "
               <<it->second << "\n";
    }
  }

  while (true) {
    foreach(MessageInstance m, view) {
      if (!rclcpp::ok())
        break;

      std::string const& topic   = m.getTopic();
      std::string callerid       = m.getCallerId();
    
      std::string callerid_topic = callerid + topic;

      if (type_missed_map.find(callerid_topic) == type_missed_map.end()) {
        if (!publish_ros1_msg(m, callerid_topic, m.getDataType()))
          throw std::runtime_error("Could'nt publish internal error");
      }
    }

    if (!rclcpp::ok()) {
      std::cout << std::endl;
      break;
    }
  }
  return true;
}

}

#endif //ROSBAG2_PLUGINS_ROS1_PLAYER_H_
