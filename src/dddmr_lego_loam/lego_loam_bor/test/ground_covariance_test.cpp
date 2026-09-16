// Regression: the single-pass moments must preserve ground-normal covariance.
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <cassert>
#include <random>

int main() {
  std::mt19937 random(7);
  std::uniform_real_distribution<float> offset(-0.2f, 0.2f);
  for (int geometry = 0; geometry < 4; ++geometry) {
    for (int trial = 0; trial < 100; ++trial) {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      std::vector<int> indices;
      for (int i = 0; i < 100; ++i) {
        pcl::PointXYZI point;
        point.x = 40 + offset(random);
        point.y = 10 + offset(random);
        point.z = geometry == 0 ? 0 : offset(random);
        if (geometry == 1) point.x = 40;  // Wall.
        if (geometry == 2) { point.y = 10; point.z = 0; }  // Line.
        cloud.push_back(point);
        indices.push_back(i);
      }
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto& point : cloud) mean += point.getVector3fMap().cast<double>();
      mean /= cloud.size();
      Eigen::Matrix3d expected = Eigen::Matrix3d::Zero();
      for (const auto& point : cloud) {
        const Eigen::Vector3d delta = point.getVector3fMap().cast<double>() - mean;
        expected += delta * delta.transpose();
      }
      Eigen::Matrix3d actual;
      Eigen::Vector4d centroid;
      pcl::computeMeanAndCovarianceMatrix(cloud, indices, actual, centroid);
      actual *= static_cast<double>(indices.size());
      assert((expected - actual).norm() < 1e-9);
    }
  }
}
