#include <p2p_move_base/obstacle_replan.h>
#include <cassert>
int main() {
  p2p_move_base::ObstacleReplan gate;
  assert(!gate.expired(100,10));
  assert(gate.blocked(100));
  assert(!gate.blocked(100.1));
  assert(gate.blocked(101));
  assert(!gate.expired(109.9,10));
  assert(gate.expired(110,10)); // Repeated plans do not restart timeout.
  gate.clear();
  assert(!gate.expired(200,10));
  assert(gate.blocked(200));
  assert(!gate.expired(201,10));
  // Replay repeated blocked plans: none may enter alignment, even though
  // each is a successful global-planner response with a different heading.
  using Admission=p2p_move_base::ObstacleReplan::Admission;
  gate.clear();
  for(int i=0;i<100;++i) {
    const auto a=gate.admit(300+i*.1,true,false);
    assert(a!=Admission::Align);
  }
  assert(gate.expired(310,10));
  gate.clear();
  // Missing/stale TF, odometry, or perception cannot approve a path and must
  // not be counted as a persistent obstacle, no matter how long validation waits.
  assert(gate.admit(400,false,true)==Admission::Wait);
  assert(gate.admit(500,false,false)==Admission::Wait);
  assert(!gate.expired(500,10));
  // The first valid blocked result starts the watchdog and requests replanning.
  assert(gate.admit(501,true,false)==Admission::Replan);
  assert(gate.admit(501.1,true,false)==Admission::Wait); // Throttle requests while stopped.
  assert(gate.admit(502,true,true)==Admission::Align); // Only a checked clear path permits turning.
  assert(!gate.expired(510.9,10));
  assert(gate.expired(511,10)); // Validation alone does not reset a real blocked watchdog.
  gate.clear(); // Executable translation clears it, as does starting another goal.
  assert(!gate.expired(511,10));
}
