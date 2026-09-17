#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace trajectory_generators {
struct TrackingErrors {
  bool valid = false;
  double heading = 0, lateral = 0, forward = 0;
};
// Use an ordered forward arc window, not the nearest tiny connector tangent.
// The same reference line supplies heading and cross-track error.
inline TrackingErrors trackingErrors(const std::vector<std::array<double, 2>>& path,
                                     double x, double y, double yaw,
                                     double lookahead = 0.5) {
  TrackingErrors out;
  if (path.size() < 2 || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(yaw) || !std::isfinite(lookahead) || lookahead <= 0) return out;
  std::vector<double> arc(path.size(), 0.0);
  double best = std::numeric_limits<double>::infinity(), nearest_arc = 0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (!std::isfinite(path[i][0]) || !std::isfinite(path[i][1])) return out;
    if (i == 0) continue;
    const double dx = path[i][0]-path[i-1][0], dy = path[i][1]-path[i-1][1];
    const double length = std::hypot(dx, dy);
    arc[i] = arc[i-1] + length;
    if (length < 1e-8) continue;
    const double t = std::clamp(((x-path[i-1][0])*dx+(y-path[i-1][1])*dy)/
                                (length*length), 0.0, 1.0);
    const double ex = path[i-1][0]+t*dx-x, ey = path[i-1][1]+t*dy-y;
    const double d2 = ex*ex+ey*ey;
    if (d2 <= best) { best = d2; nearest_arc = arc[i-1]+t*length; }
  }
  if (!std::isfinite(best) || arc.back() < 1e-8) return out;
  auto pointAt = [&](double distance) {
    for (size_t i = 1; i < path.size(); ++i) {
      const double length = arc[i]-arc[i-1];
      if (length < 1e-8 || arc[i] < distance) continue;
      const double t = std::clamp((distance-arc[i-1])/length, 0.0, 1.0);
      return std::array<double,2>{path[i-1][0]+t*(path[i][0]-path[i-1][0]),
                                  path[i-1][1]+t*(path[i][1]-path[i-1][1])};
    }
    return path.back();
  };
  const double remaining = arc.back()-nearest_arc;
  const double end = std::min(arc.back(), nearest_arc+lookahead);
  const double begin = remaining > 1e-6 ? nearest_arc+0.5*(end-nearest_arc) :
                                        std::max(0.0, end-lookahead);
  const auto a = pointAt(begin), b = pointAt(end);
  const double dx = b[0]-a[0], dy = b[1]-a[1], length = std::hypot(dx,dy);
  // A folded/degenerate window has no reliable forward direction: brake.
  if (length < 1e-6) return out;
  out.valid = true;
  out.heading = std::atan2(std::sin(std::atan2(dy,dx)-yaw),
                           std::cos(std::atan2(dy,dx)-yaw));
  out.lateral = (-dy*(a[0]-x)+dx*(a[1]-y))/length;
  out.forward = (b[0]-x)*std::cos(yaw)+(b[1]-y)*std::sin(yaw);
  return out;
}

// Sample command targets independently of near-zero measured startup velocity.
// The rollout must still integrate measured velocity with finite acceleration.
inline std::vector<double> singleAxisTargets(double lower, double upper,
    double minimum, int count, int sign, double cap) {
  std::vector<double> out;
  if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper ||
      !std::isfinite(minimum) || minimum <= 0 || !std::isfinite(cap) ||
      cap <= 0 || count < 1 || (sign != 1 && sign != -1)) return out;
  const double lo = std::max(minimum, sign > 0 ? lower : -upper);
  const double hi = std::min(cap, sign > 0 ? upper : -lower);
  if (hi < lo) return out;
  for (int i=0; i<count; ++i)
    out.push_back(sign * (count == 1 ? lo : lo+(hi-lo)*i/(count-1)));
  return out;
}

class SingleAxisTracking {
 public:
  // -1: brake; 0: X; 1: Y; 2: yaw. Hysteresis avoids stage chattering.
  int choose(const TrackingErrors& e, const std::array<double,3>& velocity,
             int64_t stamp, bool fresh, double yaw_enter, double yaw_exit,
             double lateral_enter, double lateral_exit, bool allow_yaw_translation = false) {
    if (!e.valid || !fresh || !std::isfinite(e.forward) ||
        !std::isfinite(e.heading) || !std::isfinite(e.lateral)) return reset();
    for (double v : velocity) if (!std::isfinite(v)) return reset();
    if (std::abs(e.heading) > yaw_enter) rotating_ = true;
    else if (std::abs(e.heading) <= yaw_exit) rotating_ = false;
    if (std::abs(e.lateral) > lateral_enter) lateral_ = true;
    else if (std::abs(e.lateral) <= lateral_exit) lateral_ = false;
    int wanted = rotating_ ? 2 : lateral_ ? 1 : 0;
    double error = wanted == 2 ? e.heading : wanted == 1 ? e.lateral : e.forward;
    int sign = error >= 0 ? 1 : -1;
    bool other_moving = false, stopped = true;
    for (int i=0; i<3; ++i) {
      bool moving = std::abs(velocity[i]) > (i==2 ? 0.05 : 0.03);
      stopped &= !moving;
      // Calibrated continuous yaw tolerates the known translational coupling
      // only within the same turn. Switching axis/sign still requires a stop.
      if (i != wanted && !(allow_yaw_translation && wanted == 2 &&
                           active_ == 2 && sign == sign_)) other_moving |= moving;
    }
    if (wanted != active_ || sign != sign_ || other_moving) {
      if (wanted != pending_ || sign != pending_sign_ || !stopped) count_ = 0;
      pending_ = wanted; pending_sign_ = sign;
      if (stopped && stamp > last_stamp_) ++count_;
      last_stamp_ = stamp;
      if (count_ < 3) return -1;
      active_ = wanted; sign_ = sign;
    }
    count_ = 0;
    last_stamp_ = stamp;
    return wanted;
  }
  int sign() const { return sign_; }
 private:
  int reset() { active_ = -1; pending_ = -1; count_ = 0; last_stamp_ = 0;
                rotating_ = lateral_ = false; return -1; }
  bool rotating_ = false, lateral_ = false;
  int active_ = -1, pending_ = -1, sign_ = 1, pending_sign_ = 1, count_ = 0;
  int64_t last_stamp_ = 0;
};
}
