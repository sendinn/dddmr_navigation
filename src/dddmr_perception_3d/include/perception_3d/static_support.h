#pragma once
#include <cmath>

namespace perception_3d {
inline bool inStaticSupportCylinder(double dx, double dy, double dz,
                                    double radius, double min_height, double max_height) {
  return std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz) &&
         dx*dx + dy*dy <= radius*radius && dz >= min_height && dz <= max_height;
}
inline bool rejectStaticSupport(double value, double inscribed_radius,
                               int count, int minimum) {
  return value < inscribed_radius && count < minimum;
}
}  // namespace perception_3d
