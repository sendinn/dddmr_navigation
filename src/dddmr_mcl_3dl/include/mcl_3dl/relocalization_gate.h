#pragma once
#include <cmath>
#include <cstdint>

namespace mcl_3dl {
// An anchor needs three distinct, consecutive, fresh and converged matches.
class RelocalizationGate {
 public:
  void reset() { count_ = 0; last_stamp_ = 0; }
  bool observe(int64_t stamp, double age, double x, double y, double yaw,
               double xy_limit, double yaw_limit) {
    const bool valid = stamp > last_stamp_ && stamp > 0 &&
      std::isfinite(age) && age >= 0 && age < 1.0 &&
      std::isfinite(xy_limit) && xy_limit > 0 &&
      std::isfinite(yaw_limit) && yaw_limit > 0 &&
      std::isfinite(x) && x >= 0 && x <= xy_limit * xy_limit &&
      std::isfinite(y) && y >= 0 && y <= xy_limit * xy_limit &&
      std::isfinite(yaw) && yaw >= 0 && yaw <= yaw_limit * yaw_limit;
    last_stamp_ = stamp;
    count_ = valid ? count_ + 1 : 0;
    return count_ >= 3;
  }
 private:
  int count_ = 0;
  int64_t last_stamp_ = 0;
};
}
