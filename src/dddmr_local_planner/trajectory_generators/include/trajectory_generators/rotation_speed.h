#pragma once
#include <algorithm>
#include <cmath>

namespace trajectory_generators {
// Speed magnitude; direction selection and stopping tolerance remain with the controller.
inline double rotationSpeed(double error, double minimum, double maximum, double gain = 1.0) {
  if (!std::isfinite(error)) return 0.0;
  return std::clamp(gain * std::abs(error), minimum, maximum);
}
}
