#include <p2p_move_base/rotation_pulse.h>
#include <cassert>
using p2p_move_base::RotationPulse;
int main() {
  using p2p_move_base::predictedRotationDuration;
  const double measured = 14.353481896962267*std::acos(-1.0)/180;
  assert(std::abs(predictedRotationDuration(measured, measured, .5, 2)-.5)<1e-12);
  assert(std::abs(predictedRotationDuration(-2*measured, measured, .5, 2)-1)<1e-12);
  assert(predictedRotationDuration(10, measured, .5, 2)==2);
  assert(predictedRotationDuration(0, measured, .5, 2)==0);
  assert(predictedRotationDuration(NAN, measured, .5, 2)==0);
  assert(predictedRotationDuration(1, 0, .5, 2)==0);
  RotationPulse p;
  assert(p.poll(1,.25,1,0,0,0)==RotationPulse::Idle);
  p.start(1,1);
  assert(p.poll(1.2,.25,2,0,.1,.3)==RotationPulse::Turning);
  assert(p.poll(1.3,.25,3,0,.1,.1)==RotationPulse::Braking);
  assert(p.poll(1.4,.25,4,0,0,0)==RotationPulse::Braking);
  assert(p.poll(1.5,.25,4,0,0,0)==RotationPulse::Braking);
  assert(p.poll(1.6,.25,5,0,.04,0)==RotationPulse::Braking); // Forward drift resets settling.
  assert(p.poll(1.7,.25,6,0,0,0)==RotationPulse::Braking);
  assert(p.poll(1.8,.25,7,0,0,0)==RotationPulse::Braking);
  assert(p.poll(1.9,.25,8,0,0,0)==RotationPulse::Settled);
  assert(!p.active);
  p.start(0,1);
  assert(p.poll(.3,1,1,0,.05,.6)==RotationPulse::Turning); // No fixed .25 s cutoff.
  assert(p.poll(1.01,1,2,0,.05,.6)==RotationPulse::Braking);
  p.start(2,-1); p.stop(2.1);
  assert(p.poll(2.2,.25,9,.6,0,0)==RotationPulse::Braking);
  assert(p.poll(7.2,.25,10,0,0,0)==RotationPulse::Timeout);
  p.reset(); p.stop(8); // Braking before the first turn.
  assert(p.poll(8.1,.25,11,0,0,0)==RotationPulse::Braking);
  assert(p.poll(8.2,.25,12,0,0,0)==RotationPulse::Braking);
  assert(p.poll(8.3,.25,13,0,0,0)==RotationPulse::Settled);
  p.start(10,1);
  assert(p.poll(10.5,10,14,0,0,.6,true)==RotationPulse::Turning);
  assert(p.poll(13,10,15,0,0,.6,true)==RotationPulse::Turning);
  p.stop(13.1); // Planner reached angular tolerance; issue zero immediately.
  assert(p.poll(13.2,10,16,0,0,.2,true)==RotationPulse::Braking);
  assert(p.poll(13.3,10,17,0,0,0,true)==RotationPulse::Braking);
  assert(p.poll(13.4,10,18,0,0,0,true)==RotationPulse::Braking);
  assert(p.poll(13.5,10,19,0,0,0,true)==RotationPulse::Settled);
  p.start(20,-1);
  assert(p.poll(30,10,20,0,0,.6,true)==RotationPulse::Timeout);
}
