#pragma once
#include <Eigen/Eigenvalues>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

namespace wall_ground_projection {
// Empirical A* cost for inferred wall-to-ground points. Ordinary ground stays
// at zero; the planner adds the neighborhood mean intensity to each edge cost.
inline constexpr float kProjectedGroundIntensity = 1.0f;

// Installation-pitch-corrected frame; geometric heuristic, not semantic walls.
// Never reuse inferred points as seeds.
template<class Point>
pcl::PointCloud<Point> project(const pcl::PointCloud<Point>& observed,
                              const pcl::PointCloud<Point>& ground) {
  pcl::PointCloud<Point> result;
  using Cloud = pcl::PointCloud<Point>;
  typename Cloud::Ptr raw(new Cloud), seeds(new Cloud), flat(new Cloud);
  const auto finite = [](const Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  };
  for (const auto& p : observed) if (finite(p)) raw->push_back(p);
  for (const auto& p : ground) if (finite(p)) {
    seeds->push_back(p); Point q = p; q.z = 0; flat->push_back(q);
  }
  if (raw->size() < 6 || seeds->size() < 6) return result;
  pcl::KdTreeFLANN<Point> wall_tree, ground_tree;
  wall_tree.setInputCloud(raw); ground_tree.setInputCloud(flat);
  const auto fit = [](const Cloud& cloud, const std::vector<int>& ids,
                      Eigen::Vector3d& mean, Eigen::Vector3d& normal) {
    if (ids.size() < 6) return false;
    mean.setZero();
    for (int id : ids) mean += cloud[id].getVector3fMap().template cast<double>();
    mean /= ids.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int id : ids) {
      Eigen::Vector3d d = cloud[id].getVector3fMap().template cast<double>() - mean;
      covariance += d*d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) return false;
    const auto v = solver.eigenvalues();
    if (v[2] <= 1e-10 || v[1] < .05*v[2] || v[0] > .05*v.sum()) return false;
    normal = solver.eigenvectors().col(0);
    for (int id : ids)
      if (std::abs(normal.dot(cloud[id].getVector3fMap().template cast<double>() - mean)) > .05)
        return false;
    return true;
  };
  for (const auto& p : *raw) {
    std::vector<int> ids; std::vector<float> distances;
    wall_tree.radiusSearch(p, .20, ids, distances);
    Eigen::Vector3d mean, normal;
    if (!fit(*raw, ids, mean, normal) || std::abs(normal.z()) > std::sin(.3)) continue;
    Point query = p; query.z = 0;
    ids.clear(); distances.clear();
    ground_tree.radiusSearch(query, .50, ids, distances);
    if (!fit(*seeds, ids, mean, normal) || std::abs(normal.z()) < std::cos(.3)) continue;
    const double z = mean.z() - (normal.x()*(p.x-mean.x()) + normal.y()*(p.y-mean.y()))/normal.z();
    const double height = p.z-z;
    if (!std::isfinite(z) || height < .10 || height > 1.0) continue;
    Point projected = p;
    projected.z = z;
    projected.intensity = kProjectedGroundIntensity;
    result.push_back(projected);
  }
  return result;
}
}  // namespace wall_ground_projection
