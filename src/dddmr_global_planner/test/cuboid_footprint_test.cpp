#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <global_planner/cuboid_footprint.h>
#include <global_planner/nanoflann_pcl.hpp>

namespace
{

nanoflann::KdTreeFLANN<pcl::PointXYZI>::Ptr makeTree(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  auto tree = std::make_shared<nanoflann::KdTreeFLANN<pcl::PointXYZI>>();
  tree->setInputCloud(cloud);
  return tree;
}

TEST(CuboidFootprint, RejectsPointOutsideOldCircleButInsideBodyFront)
{
  CuboidFootprint footprint;
  pcl::PointXYZI start;
  start.x = 0.0F;
  start.y = 0.0F;
  start.z = 0.0F;
  pcl::PointXYZI end = start;
  end.y = 0.50F;

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI obstacle;
  // Path-frame x=0.444, y=0.123: inside the 0.47 x 0.278 half-extents,
  // but 0.461 m from the centre and therefore outside the old 0.45 m circle.
  obstacle.x = -0.123F;
  obstacle.y = 0.444F;
  obstacle.z = 0.30F;
  cloud->push_back(obstacle);

  EXPECT_GT(std::hypot(obstacle.x, obstacle.y), 0.45);
  EXPECT_FALSE(cuboidFootprintSweepClear(
    start, end, footprint, makeTree(cloud), cloud->size(), true));
}

TEST(CuboidFootprint, AcceptsPointOutsideBodySide)
{
  CuboidFootprint footprint;
  pcl::PointXYZI start;
  start.x = 0.0F;
  start.y = 0.0F;
  start.z = 0.0F;
  pcl::PointXYZI end = start;
  end.y = 0.50F;

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI obstacle;
  obstacle.x = -0.30F;
  obstacle.y = 0.25F;
  obstacle.z = 0.30F;
  cloud->push_back(obstacle);

  EXPECT_TRUE(cuboidFootprintSweepClear(
    start, end, footprint, makeTree(cloud), cloud->size(), true));
}

TEST(CuboidFootprint, IgnoresFloorAndOverheadPoints)
{
  CuboidFootprint footprint;
  pcl::PointXYZI center;
  center.x = center.y = center.z = 0.0F;
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI floor;
  floor.x = floor.y = 0.0F;
  floor.z = 0.0F;
  cloud->push_back(floor);
  pcl::PointXYZI overhead = floor;
  overhead.z = 0.80F;
  cloud->push_back(overhead);

  EXPECT_TRUE(cuboidFootprintPoseClear(
    center, 0.0, footprint, makeTree(cloud), cloud->size(), true));
}

TEST(CuboidFootprint, UsesPathHeadingForNarrowCorridorPose)
{
  CuboidFootprint footprint;
  pcl::PointXYZI center;
  center.x = center.y = center.z = 0.0F;
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI obstacle;
  obstacle.x = 0.30F;
  obstacle.y = 0.0F;
  obstacle.z = 0.30F;
  cloud->push_back(obstacle);
  const auto tree = makeTree(cloud);

  // A 0-degree default points the 0.94 m body across the corridor and blocks.
  EXPECT_FALSE(cuboidFootprintPoseClear(
    center, 0.0, footprint, tree, cloud->size(), true));
  // The actual path heading is 90 degrees; only the 0.556 m width crosses it.
  EXPECT_TRUE(cuboidFootprintPoseClear(
    center, std::acos(-1.0) / 2.0, footprint, tree, cloud->size(), true));
}

TEST(CuboidFootprint, ExactGoalConnectorKeepsCommandedYaw)
{
  CuboidFootprint footprint;
  pcl::PointXYZI start;
  start.x = start.y = start.z = 0.0F;
  pcl::PointXYZI end = start;
  end.x = 0.02F;
  end.y = 0.01F;

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI obstacle;
  obstacle.x = 0.30F;
  obstacle.y = 0.0F;
  obstacle.z = 0.30F;
  cloud->push_back(obstacle);
  const auto tree = makeTree(cloud);

  // atan2 of the 2 cm graph quantization offset rotates the long body toward
  // the wall and reports a false collision.
  EXPECT_FALSE(cuboidFootprintSweepClear(
    start, end, footprint, tree, cloud->size(), true));
  // The commanded path points along +Y, where the wall lies outside the body.
  EXPECT_TRUE(cuboidFootprintSweepClearAtYaw(
    start, end, std::acos(-1.0) / 2.0,
    footprint, tree, cloud->size(), true));
}

}  // namespace
