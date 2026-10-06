#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>

namespace bunker_gps_nav {
struct RadiusOutlierConfig {
  bool enabled{false};
  double radius{0.2};
  int min_neighbors{1};
  double max_range{3.0};
};

inline bool valid_radius_outlier_config(const RadiusOutlierConfig &c) {
  return std::isfinite(c.radius) && c.radius > 0 &&
    std::isfinite(c.max_range) && c.max_range > 0 &&
    c.min_neighbors >= 1 && c.min_neighbors < std::numeric_limits<int>::max();
}

using ObstacleCloud = pcl::PointCloud<pcl::PointXYZ>;

// Input must already contain finite XYZ points in the vehicle base frame.
// Query only near points, but search the entire input for 3D neighbors. This
// preserves support from points just beyond max_range and all distant points.
// Run before downsampling, which can collapse a real object's neighboring hits.
inline ObstacleCloud::ConstPtr remove_near_outliers(
  const ObstacleCloud::ConstPtr &input, const RadiusOutlierConfig &c)
{
  if (!c.enabled || input->empty()) return input;
  auto near = pcl::make_shared<pcl::Indices>();
  near->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    const auto &p = (*input)[i];
    if (std::hypot(p.x, p.y) <= c.max_range) near->push_back(static_cast<int>(i));
  }
  if (near->empty()) return input;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setIndices(near);
  filter.setRadiusSearch(c.radius);
  filter.setMinNeighborsInRadius(c.min_neighbors);  // Excludes the query point.
  pcl::Indices inliers;
  filter.filter(inliers);

  std::vector<bool> keep(input->size(), true);
  for (int i : *near) keep[i] = false;
  for (int i : inliers) keep[i] = true;
  auto output = pcl::make_shared<ObstacleCloud>();
  output->header = input->header;
  output->sensor_origin_ = input->sensor_origin_;
  output->sensor_orientation_ = input->sensor_orientation_;
  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (keep[i]) output->push_back((*input)[i]);
  }
  return output;
}
}  // namespace bunker_gps_nav
