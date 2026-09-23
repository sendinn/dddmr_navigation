#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace trajectory_generators {
using Point = std::array<double, 2>;
using Velocity = std::array<double, 3>;
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline bool finite(const Velocity& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
struct TrackerConfig {
  double lookahead = 0.7, lateral_gain = 0.5, yaw_gain = 1.0;
  double lateral_deadband = 0.02, yaw_deadband = 0.01745329252;
  double lateral_slow_distance = 0.8, approach_gain = 0.7;
  double cruise_speed = 0.2, max_lateral_correction = 0.15;
  double k_xy = 0.0, k_xw = 0.0;
  void validate() const {
    for (double p : {lookahead, lateral_gain, yaw_gain, lateral_slow_distance,
                     approach_gain, cruise_speed, max_lateral_correction})
      if (!std::isfinite(p) || p <= 0) throw std::invalid_argument("Tracker gains/distances must be positive and finite");
    if (!std::isfinite(lateral_deadband) || lateral_deadband < 0 ||
        !std::isfinite(yaw_deadband) || yaw_deadband < 0 || yaw_deadband >= 1.5707963268 ||
        !std::isfinite(k_xy) || !std::isfinite(k_xw))
      throw std::invalid_argument("Invalid tracker deadband/coupling");
  }
};
struct Reference {
  bool valid = false;
  Point projection{}, lookahead{};
  double lateral = 0, heading = 0, remaining = 0;
  double tangent_heading = 0;  // 最近路径段方向（世界坐标系，弧度），用于诊断。
  Velocity desired{};
};
// Deadband subtracts its width, so the output is continuous at the boundary.
inline double deadband(double value, double width) {
  return std::copysign(std::max(0.0, std::abs(value)-width), value);
}
class CoupledPathTracker {
 public:
  explicit CoupledPathTracker(TrackerConfig config = {}) : config_(config) { config_.validate(); }
  const TrackerConfig& config() const { return config_; }
  Velocity predict(const Velocity& command) const {
    return {command[0]+config_.k_xy*command[1]+config_.k_xw*command[2], command[1], command[2]};
  }
  Velocity compensate(const Velocity& body) const {
    return {body[0]-config_.k_xy*body[1]-config_.k_xw*body[2], body[1], body[2]};
  }
  Velocity alignment(double error) const {
    return {0, 0, config_.yaw_gain*deadband(wrap(error), config_.yaw_deadband)};
  }
  Reference reference(const std::vector<Point>& path, double x, double y, double yaw) const {
    Reference out;
    if (path.size()<2 || !finite({x,y,yaw})) return out;
    std::vector<double> arc(path.size(),0.0);
    double best=std::numeric_limits<double>::infinity(), nearest=0;
    size_t segment=0;
    for (size_t i=0; i<path.size(); ++i) {
      if (!std::isfinite(path[i][0]) || !std::isfinite(path[i][1])) return out;
      if (!i) continue;
      const double dx=path[i][0]-path[i-1][0], dy=path[i][1]-path[i-1][1];
      const double length=std::hypot(dx,dy);
      arc[i]=arc[i-1]+length;
      if (length<1e-8) continue;
      const double t=std::clamp(((x-path[i-1][0])*dx+(y-path[i-1][1])*dy)/(length*length),0.0,1.0);
      Point q{path[i-1][0]+t*dx,path[i-1][1]+t*dy};
      const double distance=std::hypot(x-q[0],y-q[1]);
      // Earlier segment wins ties at crossings; no unbounded global progress jump.
      if (distance<best) {best=distance; nearest=arc[i-1]+t*length; segment=i; out.projection=q;}
    }
    if (!std::isfinite(best) || arc.back()<1e-8) return out;
    auto pointAt=[&](double s) {
      for (size_t i=1; i<path.size(); ++i) {
        const double length=arc[i]-arc[i-1];
        if (length<1e-8 || arc[i]<s) continue;
        const double t=std::clamp((s-arc[i-1])/length,0.0,1.0);
        return Point{path[i-1][0]+t*(path[i][0]-path[i-1][0]),
                     path[i-1][1]+t*(path[i][1]-path[i-1][1])};
      }
      return path.back();
    };
    out.remaining=arc.back()-nearest;
    out.lookahead=pointAt(std::min(arc.back(),nearest+config_.lookahead));
    const double local_length=arc[segment]-arc[segment-1];
    const double tx=(path[segment][0]-path[segment-1][0])/local_length;
    const double ty=(path[segment][1]-path[segment-1][1])/local_length;
    out.tangent_heading=std::atan2(ty,tx);
    out.lateral=tx*(y-out.projection[1])-ty*(x-out.projection[0]); // left positive
    double dx=out.lookahead[0]-out.projection[0], dy=out.lookahead[1]-out.projection[1];
    if (std::hypot(dx,dy)<1e-8) {dx=tx; dy=ty;}
    const double heading=std::atan2(dy,dx);
    out.heading=wrap(heading-yaw);
    // Stop forward progress when facing away, but retain lateral/yaw feedback.
    const double yaw_factor=std::max(0.0,std::cos(out.heading));
    const double lateral_factor=std::max(0.0,1.0-std::abs(out.lateral)/config_.lateral_slow_distance);
    const double speed=std::min(config_.cruise_speed,config_.approach_gain*out.remaining)*yaw_factor*lateral_factor;
    const double correction=std::clamp(-config_.lateral_gain*deadband(out.lateral,config_.lateral_deadband),
                                      -config_.max_lateral_correction,config_.max_lateral_correction);
    // Follow the local tangent; lookahead direction controls body orientation.
    double wx=speed*tx-correction*ty, wy=speed*ty+correction*tx;
    if (out.remaining<1e-6) {
      // Projection past the endpoint needs longitudinal correction as well.
      wx=config_.approach_gain*(path.back()[0]-x);
      wy=config_.approach_gain*(path.back()[1]-y);
    }
    out.desired={std::cos(yaw)*wx+std::sin(yaw)*wy,
                 -std::sin(yaw)*wx+std::cos(yaw)*wy,
                 config_.yaw_gain*deadband(out.heading,config_.yaw_deadband)};
    out.valid=finite(out.desired);
    return out;
  }
  // Uniform scaling preserves the compensation relation even at actuator limits.
  Velocity boundedCommand(const Velocity& desired, const Velocity& lower,
                          const Velocity& upper, double max_translation) const {
    if (!finite(desired)) return {};
    Velocity u=compensate(desired);
    double scale=1.0;
    for (size_t i=0;i<3;++i) {
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i]>0 || upper[i]<0)
        throw std::invalid_argument("Command bounds must include zero");
      if (u[i]>0) scale=std::min(scale,upper[i]/u[i]);
      if (u[i]<0) scale=std::min(scale,lower[i]/u[i]);
    }
    if (!std::isfinite(max_translation) || max_translation<=0)
      throw std::invalid_argument("Translation limit must be positive");
    const double norm=std::max(std::hypot(u[0],u[1]),std::hypot(desired[0],desired[1]));
    if (norm>max_translation) scale=std::min(scale,max_translation/norm);
    for (double& value:u) value*=scale;
    return finite(u)?u:Velocity{};
  }
 private:
  TrackerConfig config_;
};
}  // namespace trajectory_generators
