#pragma once

#include <cmath>
#include <deque>
#include <mcl_3dl/state_6dof.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mcl_3dl
{
// Bounded history; never extrapolate a lidar measurement beyond odometry.
class OdomHistory
{
public:
  bool push(const State6DOF& state)
  {
    if (!std::isfinite(state.time_stamp_) ||
        (!states_.empty() && state.time_stamp_ <= states_.back().time_stamp_)) return false;
    states_.push_back(state);
    while (states_.size() > 1000 ||
           (states_.size() > 2 && states_.back().time_stamp_ - states_.front().time_stamp_ > 3.0))
      states_.pop_front();
    return true;
  }

  bool sample(double stamp, State6DOF& result) const
  {
    if (!std::isfinite(stamp) || states_.empty() ||
        stamp < states_.front().time_stamp_ || stamp > states_.back().time_stamp_) return false;
    for (size_t i = 0; i < states_.size(); ++i) {
      const auto& right = states_[i];
      if (stamp == right.time_stamp_) { result = right; return true; }
      if (right.time_stamp_ < stamp) continue;
      const auto& left = states_[i - 1];
      // Do not invent motion across a missing odometry stream.
      if (right.time_stamp_ - left.time_stamp_ > 0.3) return false;
      const double u = (stamp - left.time_stamp_) / (right.time_stamp_ - left.time_stamp_);
      tf2::Quaternion a(left.rot_.x_, left.rot_.y_, left.rot_.z_, left.rot_.w_);
      tf2::Quaternion b(right.rot_.x_, right.rot_.y_, right.rot_.z_, right.rot_.w_);
      auto q = a.slerp(b, u); q.normalize();
      result = State6DOF(left.pos_ * (1.0 - u) + right.pos_ * u,
                        Quat(q.x(), q.y(), q.z(), q.w()), stamp);
      return true;
    }
    return false;
  }
private:
  std::deque<State6DOF> states_;
};
}  // namespace mcl_3dl
