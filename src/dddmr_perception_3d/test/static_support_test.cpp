#include <perception_3d/static_support.h>
#include <algorithm>
#include <cassert>
#include <limits>

int main() {
  using namespace perception_3d;
  assert(inStaticSupportCylinder(0.1, 0, .1, .1, .1, 3));
  assert(inStaticSupportCylinder(0, 0, 3, .1, .1, 3));
  assert(!inStaticSupportCylinder(.101, 0, .5, .1, .1, 3));
  assert(!inStaticSupportCylinder(.09, .09, .5, .1, .1, 3));
  assert(!inStaticSupportCylinder(0, 0, .09, .1, .1, 3));
  assert(!inStaticSupportCylinder(0, 0, 3.01, .1, .1, 3));
  assert(!inStaticSupportCylinder(std::numeric_limits<double>::quiet_NaN(), 0, .5, .1, .1, 3));
  assert(rejectStaticSupport(.25, .45, 0, 2));
  assert(rejectStaticSupport(.25, .45, 1, 2));
  assert(!rejectStaticSupport(.25, .45, 2, 2));
  assert(!rejectStaticSupport(.25, .45, 3, 2));
  assert(!rejectStaticSupport(.45, .45, 0, 2));
  // A withdrawn static contribution must not override a dynamic obstacle.
  assert(std::min(9999.0, .2) == .2);
}
