#pragma once
#include <cmath>
#include <algorithm>
#include <cstdint>
namespace p2p_move_base {
// One measured point gives an effective angle/time scale, including coast-down.
// It is an initial estimate, not an independently identified delay model.
inline double predictedRotationDuration(double angle, double measured_angle,
                                        double measured_duration, double maximum) {
  if (!std::isfinite(angle) || !std::isfinite(measured_angle) ||
      !std::isfinite(measured_duration) || !std::isfinite(maximum) ||
      measured_angle <= 0 || measured_duration <= 0 || maximum <= 0) return 0;
  return std::min(maximum, std::abs(angle)*measured_duration/measured_angle);
}
class RotationPulse {
 public:
  enum Result { Idle, Turning, Braking, Settled, Timeout };
  bool active=false;
  int sign=0;
  void start(double now, int direction) { active=true; sign=direction; start_=now; braking_=false; count_=0; stamp_=0; }
  void stop(double now) { if (!active) start(now,0); if (!braking_) { braking_=true; stop_=now; count_=0; stamp_=0; } }
  void reset() { active=false; braking_=false; count_=0; sign=0; }
  Result poll(double now, double duration, int64_t stamp, double age, double speed, double yaw_rate,
              bool angle_feedback = false) {
    if (!active) return Idle;
    // In feedback mode the planner calls stop() on angle convergence. Time
    // is only an abort watchdog, never a substitute for reaching the angle.
    if (!braking_ && angle_feedback && now-start_ >= duration) return Timeout;
    if (!braking_ && now-start_ >= duration) stop(now);
    if (!braking_) return Turning;
    if (now-stop_ >= 5.0) return Timeout;
    bool stopped = stamp>0 && age>=0 && age<0.5 && std::isfinite(speed) &&
      std::isfinite(yaw_rate) && speed<=0.03 && std::abs(yaw_rate)<=0.05;
    if (!stopped || stamp<stamp_) count_=0;
    else if (stamp>stamp_) ++count_;
    stamp_=stamp;
    if (count_>=3) { reset(); return Settled; }
    return Braking;
  }
 private:
  bool braking_=false;
  double start_=0,stop_=0;
  int count_=0;
  int64_t stamp_=0;
};
}
