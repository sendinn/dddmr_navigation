#ifndef GLOBAL_PLANNER__CUBOID_FOOTPRINT_H_
#define GLOBAL_PLANNER__CUBOID_FOOTPRINT_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct CuboidFootprint
{
  double front{0.47};
  double back{0.47};
  double left{0.278};
  double right{0.278};
  // Global path poses lie on the ground. Ignore the floor itself, then check
  // the complete body height above it (the 0.66 m specification excludes the antenna).
  double bottom{0.10};
  double top{0.66};
  double sample_step{0.05};
};

inline bool pointInsideCuboidFootprint(
  double local_x, double local_y, double local_z,
  const CuboidFootprint & footprint, bool check_height)
{
  constexpr double epsilon = 1e-6;
  const bool inside_xy =
    local_x >= -footprint.back - epsilon &&
    local_x <= footprint.front + epsilon &&
    local_y >= -footprint.right - epsilon &&
    local_y <= footprint.left + epsilon;
  if (!inside_xy || !check_height) {
    return inside_xy;
  }
  return local_z >= footprint.bottom - epsilon &&
         local_z <= footprint.top + epsilon;
}

template<typename TreePtr>
// 检查车体包围盒在某一个固定姿态下是否没有障碍点。
// center 是包围盒的全局坐标，yaw 是车体前向在 XY 平面内的朝向。
// 先用 KD-tree 球形邻域做粗筛，再把候选障碍转到车体坐标系做精确长方体判断。
inline bool cuboidFootprintPoseClear(
  const pcl::PointXYZI & center, double yaw,
  const CuboidFootprint & footprint, const TreePtr & obstacle_tree,
  std::size_t obstacle_point_count, bool check_height)
{
  // 没有搜索树或本层没有障碍点时，该数据层不会产生碰撞。
  if (!obstacle_tree || obstacle_point_count == 0) {
    return true;
  }

  // 预先计算旋转系数，用于将障碍点从全局 XY 坐标旋转到车体坐标系。
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  // 取车体前后、左右和上下的最大绝对范围，构造一个能完全包住长方体的球。
  // 这个球只用于 KD-tree 候选点搜索，不是最终的碰撞形状。
  const double longitudinal = std::max(footprint.front, footprint.back);
  const double lateral = std::max(footprint.left, footprint.right);
  // check_height=false 时不考虑 Z，搜索半径只需覆盖 XY 占地矩形。
  const double vertical = check_height ?
    std::max(std::abs(footprint.bottom), std::abs(footprint.top)) : 0.0;
  // 外接球半径 = sqrt(longitudinal^2 + lateral^2 + vertical^2)。
  const double search_radius = std::hypot(std::hypot(longitudinal, lateral), vertical);

  // 找出外接球内的障碍候选点。squared_distances 由 radiusSearch 返回，
  // 本函数只需要点索引，不再使用距离值。
  std::vector<int> indices;
  std::vector<float> squared_distances;
  obstacle_tree->radiusSearch(center, search_radius, indices, squared_distances);
  // 逐个对球形粗筛结果做精确包围盒检查。
  for (const int index : indices) {
    const auto & obstacle = obstacle_tree->getInputCloud()->points[index];
    // 先平移，得到障碍点相对车体中心的全局坐标差。
    const double obstacle_dx = obstacle.x - center.x;
    const double obstacle_dy = obstacle.y - center.y;
    // 再旋转 -yaw，转换到车体坐标系：+x 向前，+y 向左。
    const double local_x = obstacle_dx * cos_yaw + obstacle_dy * sin_yaw;
    const double local_y = -obstacle_dx * sin_yaw + obstacle_dy * cos_yaw;
    // Z 轴不参与 yaw 旋转，只计算相对车体中心的高度。
    const double local_z = obstacle.z - center.z;
    // 点落入 [-back, front] x [-right, left] 内时命中 XY 包围盒；
    // check_height=true 时还必须同时落入 [bottom, top] 才判定碰撞。
    if (pointInsideCuboidFootprint(
        local_x, local_y, local_z, footprint, check_height))
    {
      // 任意一个障碍点进入车体包围盒，当前姿态即不安全。
      return false;
    }
  }
  // 所有候选障碍点都在包围盒外，当前姿态通过检查。
  return true;
}

template<typename TreePtr>
// 检查车体长方体沿 start -> end 平移时是否与障碍点碰撞。
// 车体 yaw 始终取路径的 XY 方向；沿线段离散采样后，
// 由 cuboidFootprintPoseClear() 检查每个采样姿态。
inline bool cuboidFootprintSweepClear(
  const pcl::PointXYZI & start, const pcl::PointXYZI & end,
  const CuboidFootprint & footprint, const TreePtr & obstacle_tree,
  std::size_t obstacle_point_count, bool check_height)
{
  // 没有搜索树或障碍点时，本数据层不会造成碰撞。
  if (!obstacle_tree || obstacle_point_count == 0) {
    return true;
  }

  // 计算候选边在三个坐标轴上的位移。
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double dz = end.z - start.z;
  // 采样数与车体朝向只由 XY 平面距离决定，Z 在插值时单独处理。
  const double planar_distance = std::hypot(dx, dy);
  // 当起终点 XY 基本重合时，现有逻辑直接认为该“扫掠”无碰撞，
  // 不会在起点或终点额外调用姿态碰撞检查。
  if (planar_distance <= 1e-6) {
    return true;
  }

  // 将车体前向对齐候选边的 XY 移动方向。
  const double yaw = std::atan2(dy, dx);
  // 按 sample_step 确定区间数，ceil 保证实际采样间隔不超过设定步长。
  // 至少划分一个区间，后面会同时检查起点和终点。
  const int steps = std::max(
    1, static_cast<int>(std::ceil(planar_distance / footprint.sample_step)));

  // step 从 0 到 steps，因此起点、中间采样点和终点都会被检查。
  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    pcl::PointXYZI center;
    // 在候选边上做线性插值，得到当前车体包围盒中心。
    center.x = start.x + dx * ratio;
    center.y = start.y + dy * ratio;
    center.z = start.z + dz * ratio;
    // intensity 不参与几何碰撞计算，仅将查询点初始化为确定值。
    center.intensity = 0.0F;
    // check_height=true 时检查完整三维长方体；false 时只检查 XY 占地范围。
    // 任意一个采样姿态碰撞，整条候选边立即判定为不可通过。
    if (!cuboidFootprintPoseClear(
        center, yaw, footprint, obstacle_tree, obstacle_point_count, check_height))
    {
      return false;
    }
  }
  // 所有采样姿态都通过后，才认为这条候选边无碰撞。
  return true;
}

// Translate the footprint while retaining a commanded/path yaw.  This is
// useful for graph-to-exact-goal connectors: their small displacement is a
// point-cloud quantization offset, so atan2(displacement) is not a meaningful
// robot heading.
template<typename TreePtr>
inline bool cuboidFootprintSweepClearAtYaw(
  const pcl::PointXYZI & start, const pcl::PointXYZI & end, double yaw,
  const CuboidFootprint & footprint, const TreePtr & obstacle_tree,
  std::size_t obstacle_point_count, bool check_height)
{
  if (!obstacle_tree || obstacle_point_count == 0) {
    return true;
  }

  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double dz = end.z - start.z;
  const double planar_distance = std::hypot(dx, dy);
  const int steps = std::max(
    1, static_cast<int>(std::ceil(planar_distance / footprint.sample_step)));

  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    pcl::PointXYZI center;
    center.x = start.x + dx * ratio;
    center.y = start.y + dy * ratio;
    center.z = start.z + dz * ratio;
    center.intensity = 0.0F;
    if (!cuboidFootprintPoseClear(
        center, yaw, footprint, obstacle_tree, obstacle_point_count, check_height))
    {
      return false;
    }
  }
  return true;
}

#endif  // GLOBAL_PLANNER__CUBOID_FOOTPRINT_H_
