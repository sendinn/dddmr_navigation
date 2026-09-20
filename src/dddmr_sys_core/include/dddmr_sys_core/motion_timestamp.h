#pragma once
#include <cmath>
#include <cstdint>

namespace dddmr_sys_core {
// Device-to-ROS clock mapping can lead the host clock by a few milliseconds.
// Accept only bounded skew; never rewrite stamps or extend the stale limit.
inline bool motionTimestampFresh(int64_t stamp, double age) {
  constexpr double future_tolerance = 0.020;
  constexpr double stale_limit = 0.5;
  return stamp > 0 && std::isfinite(age) &&
         age >= -future_tolerance && age < stale_limit;
}
}
