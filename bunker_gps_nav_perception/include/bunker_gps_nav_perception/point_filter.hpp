#pragma once
#include <cmath>

namespace bunker_gps_nav {
struct PointFilterConfig {
  double body_min_x{-0.65}, body_max_x{0.65};
  double body_min_y{-0.45}, body_max_y{0.45};
  double body_min_z{-0.30}, body_max_z{0.60};
  double min_height{0.05}, max_height{1.50};
  double min_range{0.30}, max_range{50.0};
};
enum class PointResult { Keep, NonFinite, SelfBody, Height, Range };

// Evaluate in the vehicle base frame. Box surfaces are excluded too.
inline PointResult classify_point(double x, double y, double z, const PointFilterConfig& c) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return PointResult::NonFinite;
  if (x >= c.body_min_x && x <= c.body_max_x &&
      y >= c.body_min_y && y <= c.body_max_y &&
      z >= c.body_min_z && z <= c.body_max_z) return PointResult::SelfBody;
  if (z < c.min_height || z > c.max_height) return PointResult::Height;
  const double range = std::hypot(x,y);
  if (range < c.min_range || range > c.max_range) return PointResult::Range;
  return PointResult::Keep;
}
}  // namespace bunker_gps_nav
