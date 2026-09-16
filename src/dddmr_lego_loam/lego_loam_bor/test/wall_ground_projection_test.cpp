#include "wall_ground_projection.h"
#include <pcl/point_types.h>
#include <cassert>

int main() {
  using Point = pcl::PointXYZI;
  pcl::PointCloud<Point> wall, ground, empty;
  for (int i=-8; i<=8; ++i) for (int j=0; j<10; ++j) {
    Point p; p.x=0; p.y=i*.05f; p.z=.2f+j*.05f; p.intensity=1;
    wall.push_back(p);
  }
  for (int i=1; i<9; ++i) for (int j=-10; j<=10; ++j) {
    Point p; p.x=i*.05f; p.y=j*.05f; p.z=0; p.intensity=0;
    ground.push_back(p);
  }
  const auto projected = wall_ground_projection::project(wall, ground);
  assert(!projected.empty());
  for (const auto& p : projected) assert(std::abs(p.z)<1e-5 && std::abs(p.x)<1e-5);
  assert(wall_ground_projection::project(wall, empty).empty());
  assert(wall_ground_projection::project(ground, ground).empty());
  auto distant=ground;
  for (auto& p : distant) p.x+=5;
  assert(wall_ground_projection::project(wall, distant).empty());
  auto shifted_wall=wall, shifted_ground=ground;
  for (auto& p : shifted_wall) p.z-=2;
  for (auto& p : shifted_ground) p.z-=2;
  const auto shifted=wall_ground_projection::project(shifted_wall, shifted_ground);
  assert(!shifted.empty());
  for (const auto& p : shifted) assert(std::abs(p.z+2)<1e-5);
  for (auto& p : shifted_wall) p.z+=5;
  assert(wall_ground_projection::project(shifted_wall, shifted_ground).empty());
}
