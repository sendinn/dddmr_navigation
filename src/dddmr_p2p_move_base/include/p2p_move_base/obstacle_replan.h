#pragma once
namespace p2p_move_base {
// Steady-clock watchdog spans planning/alignment states; accepting an unchanged
// global path or a zero command must not restart the blocked deadline.
class ObstacleReplan {
 public:
  enum class Admission { Wait, Replan, Align };
  Admission admit(double now, bool data_valid, bool path_clear) {
    if (data_valid && path_clear) return Admission::Align;
    if (!active_) { active_=true; started_=now; }
    if (data_valid && blocked(now)) return Admission::Replan;
    return Admission::Wait;
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
