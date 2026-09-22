#include "wall_ground_projection.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <cassert>

int main() {
  using Point = pcl::PointXYZI;
  pcl::PointCloud<Point> wall, ground, empty;
  for (int i=-8; i<=8; ++i) for (int j=0; j<10; ++j) {
    Point p; p.x=0; p.y=i*.05f; p.z=.2f+j*.05f; p.intensity=42;
    wall.push_back(p);
  }
  for (int i=1; i<9; ++i) for (int j=-10; j<=10; ++j) {
    Point p; p.x=i*.05f; p.y=j*.05f; p.z=0; p.intensity=0;
    ground.push_back(p);
  }
  const auto projected = wall_ground_projection::project(wall, ground);
  assert(!projected.empty());
  for (const auto& p : projected)
    assert(p.intensity == wall_ground_projection::kProjectedGroundIntensity);
  for (const auto& p : ground) assert(p.intensity == 0);
  for (const auto& p : wall) assert(p.intensity == 42);
  // Ground generation and map loading both voxel-filter all fields. Pure
  // projected voxels retain their cost; mixed ground voxels average it.
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(.1f, .1f, .1f);
  filter.setDownsampleAllData(true);
  filter.setInputCloud(projected.makeShared());
  pcl::PointCloud<Point> sampled;
  filter.filter(sampled);
  assert(!sampled.empty());
  filter.setLeafSize(.2f, .2f, .2f);
  filter.setInputCloud(sampled.makeShared());
  pcl::PointCloud<Point> loaded;
  filter.filter(loaded);
  assert(!loaded.empty());
  for (const auto& p : loaded)
    assert(std::abs(p.intensity - wall_ground_projection::kProjectedGroundIntensity) < 1e-5);
  pcl::PointCloud<Point> mixed;
  Point low, high;
  low.x=low.y=low.z=.01f; low.intensity=0;
  high=low; high.x=.02f; high.intensity=wall_ground_projection::kProjectedGroundIntensity;
  mixed.push_back(low); mixed.push_back(high);
  filter.setInputCloud(mixed.makeShared());
  filter.filter(loaded);
  assert(loaded.size() == 1);
  assert(std::abs(loaded[0].intensity - .5f * wall_ground_projection::kProjectedGroundIntensity) < 1e-5);
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
