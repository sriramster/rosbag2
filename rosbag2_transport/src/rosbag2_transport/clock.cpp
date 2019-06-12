#include "rosbag2_transport/clock.hpp"
#include "rosbag2_transport/logging.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <chrono>

using namespace std::chrono;

namespace rosbag2_transport {

using Time = std::chrono::time_point<std::chrono::high_resolution_clock>;

TimePublisher::TimePublisher()
    :Node("test_clock")
{
  time_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", rmw_qos_profile_default);
  rclcpp::WallRate loop_rate(30.0);
}

TimePublisher::~TimePublisher()
{
  // auto-generated
}

void TimePublisher::enableClock(bool en)
{
  do_publish_ = en;
}

void TimePublisher::setHorizon(const TimePoint& time)
{
  horizon_ = time;
}

void TimePublisher::setWCHorizon(const TimePoint& time)
{
  wc_horizon_ = time;
}
     
void TimePublisher::setTime(const TimePoint& time)
{
  current_ = time;
}

TimePoint const& TimePublisher::getTime() const
{
  return current_;
}

void TimePublisher::publishClock(const TimePoint & t)
{
  rosgraph_msgs::msg::Clock pub_msg;

  auto secs = time_point_cast<seconds>(t);
  auto nsecs = time_point_cast<nanoseconds>(t) - time_point_cast<nanoseconds>(secs);

  pub_msg.clock.sec = secs.time_since_epoch().count();
  pub_msg.clock.nanosec = nsecs.count();

  time_pub_->publish(pub_msg);
}
     
// Publish clock by default
void TimePublisher::runClock(const std::chrono::nanoseconds& time_since_start_)
{
  (void)time_since_start_;
  rosgraph_msgs::msg::Clock pub_msg;

  Time t = std::chrono::high_resolution_clock::now();
  Time done = t + std::chrono::milliseconds(100);

  // std::time_t now_c = std::chrono::high_resolution_clock::to_time_t(wc_horizon_);
  // std::cout<<"1 :"<<now_c<<std::endl;

  // now_c = std::chrono::high_resolution_clock::to_time_t(t);
  // std::cout<<"2:"<<now_c<<std::endl;

  // std::chrono::duration<double> time_span = duration_cast<duration<double>>(t - wc_horizon_);
  // std::cout << "It took me " << time_span.count() << " seconds."<<std::endl;

  while (t < done) //&& t < wc_horizon_)
  {
    auto leftHorizonWC = wc_horizon_ - t;

    current_ = horizon_ - leftHorizonWC;

    if (current_ >= horizon_)
      current_ = horizon_;

    if (t >= next_pub_)
    {
      // Convert time_point to timespec before publishing
      auto secs = time_point_cast<seconds>(current_);
      auto nsecs = time_point_cast<nanoseconds>(current_) - time_point_cast<nanoseconds>(secs);

      pub_msg.clock.sec = secs.time_since_epoch().count();
      pub_msg.clock.nanosec = nsecs.count();

      time_pub_->publish(pub_msg);
      next_pub_ = t + std::chrono::milliseconds(10);
    }

    Time target = done;
    if (target > wc_horizon_)
      target = wc_horizon_;
    if (target > next_pub_)
      target = next_pub_;

    std::this_thread::sleep_until(target);

    t = std::chrono::system_clock::now();
  }
}

} // namespace rosbag2_transport 
