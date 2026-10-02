#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <GeographicLib/Geocentric.hpp>
#include <ublox_msgs/msg/nav_relposned9.hpp>

namespace bunker_gps_nav {
constexpr double pi = 3.14159265358979323846;
enum class Rtk : uint8_t { INVALID = 0, FLOAT = 1, FIXED = 2 };
inline Rtk rtk_state(uint32_t flags) {
  using M = ublox_msgs::msg::NavRELPOSNED9;
  if (!(flags & M::FLAGS_GNSS_FIX_OK) || !(flags & M::FLAGS_REL_POS_VALID)) return Rtk::INVALID;
  switch (flags & M::FLAGS_CARR_SOLN_MASK) {
    case M::FLAGS_CARR_SOLN_FIXED: return Rtk::FIXED;
    case M::FLAGS_CARR_SOLN_FLOAT: return Rtk::FLOAT;
    default: return Rtk::INVALID;
  }
}
inline bool heading_valid(uint32_t flags) {
  return rtk_state(flags) != Rtk::INVALID &&
    (flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_HEAD_VALID);
}
inline double heading_to_yaw(double heading_deg, double mount_offset_deg = 0.0) {
  return std::remainder(pi / 2.0 - heading_deg * pi / 180.0 + mount_offset_deg * pi / 180.0, 2.0 * pi);
}
inline bool valid_lla(const std::array<double, 3>& p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
    std::abs(p[0]) <= 90.0 && std::abs(p[1]) <= 180.0;
}
inline std::array<double, 3> midpoint(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  if (!valid_lla(a) || !valid_lla(b)) throw std::invalid_argument("Invalid LLA");
  const auto& earth = GeographicLib::Geocentric::WGS84();
  double ax, ay, az, bx, by, bz;
  earth.Forward(a[0], a[1], a[2], ax, ay, az);
  earth.Forward(b[0], b[1], b[2], bx, by, bz);
  std::array<double, 3> result;
  earth.Reverse((ax+bx)/2, (ay+by)/2, (az+bz)/2, result[0], result[1], result[2]);
  return result;
}
}  // namespace bunker_gps_nav
