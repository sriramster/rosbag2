#ifndef ROSBAG2_TRANSPORT__TRANSLATE_HPP_
#define ROSBAG2_TRANSPORT__TRANSLATE_HPP_

#include <chrono>

namespace rosbag2_transport {

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

class TimeTranslator
{
 public:
  TimeTranslator();

  void      setTimeScale(double const& s);
  void      setRealStartTime(TimePoint const& t);
  void      setTranslatedStartTime(TimePoint const& t);
  // void      shift(rclcpp::Duration const& d);
  TimePoint translate(TimePoint const& t);

 private:
  double    time_scale_;
  TimePoint real_start_;
  TimePoint translated_start_;
};     

}

#endif // ROSBAG2_TRANSPORT__TRANSLATE_HPP_
