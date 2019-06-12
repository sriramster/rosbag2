#ifndef ROSBAG2_TRANSPORT__CLOCK_HPP_
#define ROSBAG2_TRANSPORT__CLOCK_HPP_

#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

namespace rosbag2_transport {

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
using Duration = std::chrono::duration<long double, std::ratio<1,1000000>>;

class TimePublisher : public rclcpp::Node {
public:
  /*! Create a time publisher
   *  A publish_frequency of < 0 indicates that time shouldn't actually be published
   */
  TimePublisher();

  virtual ~TimePublisher();
  
  void enableClock(bool en);

  /*! Set the current time */
  void setTime(const TimePoint& time);

  /*! Get the current time */
  TimePoint const& getTime() const;

  void setHorizon(const TimePoint& time);

  void setWCHorizon(const TimePoint& time);

  void runClock(const std::chrono::nanoseconds& time_since_start_);

  void publishClock(const TimePoint & t);

private:
  bool do_publish_;

  double publish_frequency_;
  double time_scale_;
    
  //rclcpp::Duration wall_step_;
    
  TimePoint next_pub_;
  TimePoint current_;
  TimePoint horizon_, wc_horizon_;
  
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr time_pub_;
};

} // namespace rosbag2_transport

#endif //ROSBAG2_TRANSPORT__CLOCK_HPP_
