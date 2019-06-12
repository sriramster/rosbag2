#include <rosbag2_transport/translate.hpp>

#include <chrono>
#include <iostream>

namespace rosbag2_transport {
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

TimeTranslator::TimeTranslator()
    : time_scale_(1.0)
{
          
  real_start_ = std::chrono::time_point<std::chrono::high_resolution_clock>::min();
  translated_start_ = std::chrono::time_point<std::chrono::high_resolution_clock>::min();
}

void TimeTranslator::setTimeScale(double const& s) {
  time_scale_ = s;
}

void TimeTranslator::setRealStartTime(TimePoint const& t) {
  real_start_ = t;
}

void TimeTranslator::setTranslatedStartTime(TimePoint const& t) {
  translated_start_ = t;
}

// Ignoring timescale for now
TimePoint TimeTranslator::translate(TimePoint const& t) {
  return translated_start_ + (t - real_start_);
}

}
