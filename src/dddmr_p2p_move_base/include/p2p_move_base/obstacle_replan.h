#pragma once
namespace p2p_move_base {
// Steady-clock watchdog spans planning/alignment states after a valid path check
// has confirmed blockage. Missing/stale TF, odometry, or perception data only
// pauses validation and must not start the blocked deadline. Accepting an
// unchanged global path or a zero command must not restart that deadline.
class ObstacleReplan {
 public:
  enum class Admission { Wait, Replan, Align };
  Admission admit(double now, bool data_valid, bool path_clear) {
    if (!data_valid) return Admission::Wait;
    if (path_clear) return Admission::Align;
    return blocked(now) ? Admission::Replan : Admission::Wait;
  }
  void clear() { active_=false; last_request_=-1; }
  bool blocked(double now) {
    if(!active_) {active_=true; started_=now;}
    if(last_request_<0 || now-last_request_>=1.0) {last_request_=now;return true;}
    return false;
  }
  bool expired(double now,double timeout) const {return active_ && now-started_>=timeout;}
 private:
  bool active_=false;
  double started_=0,last_request_=-1;
};
}
