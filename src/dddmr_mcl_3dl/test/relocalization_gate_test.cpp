#include <mcl_3dl/relocalization_gate.h>
#include <cassert>
#include <limits>
int main() {
  mcl_3dl::RelocalizationGate gate;
  auto sample = [&](int64_t stamp, double age = .1, double x = .0025) {
    return gate.observe(stamp, age, x, .0025, .0025, .15, .2);
  };
  assert(!sample(1)); assert(!sample(2)); assert(sample(3));
  gate.reset();
  assert(!sample(4)); assert(!sample(4)); assert(!sample(5));
  assert(!sample(6)); assert(sample(7));
  gate.reset();
  assert(!sample(8)); assert(!sample(9, 1.0));
  assert(!sample(10)); assert(!sample(11)); assert(sample(12));
  gate.reset();
  assert(!sample(13)); assert(!sample(14, .1, .1));
  assert(!sample(15)); assert(!sample(16)); assert(sample(17));
  assert(!sample(18, -.01));
  assert(!sample(19, .1, std::numeric_limits<double>::quiet_NaN()));
  gate.reset(); assert(!sample(20));  // Next initialpose invalidates previous success.
}
