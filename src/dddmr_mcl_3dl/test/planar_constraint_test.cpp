#include <cassert>
#include <cmath>
#include <mcl_3dl/planar_constraint.h>

int main()
{
  using namespace mcl_3dl;
  Quat rotation;
  rotation.setRPY(Vec3(0.2, -0.3, 0.7));
  State6DOF s(Vec3(1.2, -0.4, -0.51), rotation, 0.0);
  constrainPlanarState(s, -0.08);
  assert(std::abs(s.pos_.z_ + 0.08) < 1e-6);
  assert(std::abs(s.pos_.x_ - 1.2) < 1e-6);
  assert(std::abs(s.pos_.y_ + 0.4) < 1e-6);
  auto rpy = s.rot_.getRPY();
  assert(std::abs(rpy.x_) < 1e-6 && std::abs(rpy.y_) < 1e-6);
  assert(std::abs(rpy.z_ - 0.7) < 1e-6);
  // Reinitialization uses the newly supplied body height, never a floor z.
  constrainPlanarState(s, 0.15);
  assert(std::abs(s.pos_.z_ - 0.15) < 1e-6);
  constrainPlanarState(s, 0.15);
  assert(std::abs(s.rot_.getRPY().z_ - 0.7) < 1e-6);
}
