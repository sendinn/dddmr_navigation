#pragma once

#include <mcl_3dl/state_6dof.h>

namespace mcl_3dl
{
// Flat-floor mode constrains the robot origin, NOT the mapground elevation.
inline void constrainPlanarState(State6DOF& state, double base_z)
{
  const auto rpy = state.rot_.getRPY();
  state.pos_.z_ = base_z;
  state.rot_.setRPY(Vec3(0.0, 0.0, rpy.z_));
}
}
