#include "p2p_move_base/localization_recovery.h"
#include <cassert>
using p2p_move_base::LocalizationRecovery;
int main() {
  LocalizationRecovery gate;
  constexpr int64_t s = 1000000000;
  assert(!gate.ready(1*s, 2*s, 1.0));
  gate.observe(true, 1*s, 1*s, 0);
  assert(!gate.ready(2*s, 3*s, 1.0)); // No distinct new frame.
  gate.observe(true, 15*s/10, 15*s/10, 2*s);
  assert(!gate.ready(19*s/10, 25*s/10, 1.0));
  assert(gate.ready(2*s, 25*s/10, 1.0));
  assert(!gate.ready(25*s/10, 25*s/10, 1.0)); // Expired.
  gate.observe(false, 21*s/10, 21*s/10, 25*s/10);
  gate.observe(true, 22*s/10, 22*s/10, 0);
  assert(!gate.ready(3*s, 32*s/10, 1.0)); // Invalid frame resets hold.
  gate.observe(true, 4*s, 4*s, 32*s/10);
  gate.observe(true, 45*s/10, 45*s/10, 5*s);
  assert(!gate.ready(49*s/10, 55*s/10, 1.0)); // Gap resets hold.
  assert(gate.ready(5*s, 55*s/10, 1.0));
  gate.observe(true, 51*s/10, 3*s, 55*s/10);
  assert(!gate.ready(52*s/10, 6*s, 1.0)); // Reordered stamp resets hold.
}
