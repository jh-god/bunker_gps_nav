#include <gtest/gtest.h>
#include <GeographicLib/LocalCartesian.hpp>
#include "bunker_gps_nav_localization/gnss_math.hpp"
using namespace bunker_gps_nav;
TEST(Heading, CardinalDirections) {
  EXPECT_NEAR(heading_to_yaw(0),pi/2,1e-12);
  EXPECT_NEAR(heading_to_yaw(90),0,1e-12);
  EXPECT_NEAR(heading_to_yaw(180),-pi/2,1e-12);
  EXPECT_NEAR(std::abs(heading_to_yaw(270)),pi,1e-12);
  EXPECT_NEAR(heading_to_yaw(0,-90),0,1e-12);
  EXPECT_NEAR(heading_to_yaw(450),0,1e-12);
}
TEST(Flags, IndependentBits) {
  using M=ublox_msgs::msg::NavRELPOSNED9;
  EXPECT_EQ(rtk_state(311),Rtk::FIXED);
  EXPECT_TRUE(heading_valid(311));
  EXPECT_EQ(rtk_state(311 & ~M::FLAGS_GNSS_FIX_OK),Rtk::INVALID);
  EXPECT_EQ(rtk_state(311 & ~M::FLAGS_REL_POS_VALID),Rtk::INVALID);
  EXPECT_FALSE(heading_valid(311 & ~M::FLAGS_REL_POS_HEAD_VALID));
  EXPECT_EQ(rtk_state((311 & ~M::FLAGS_CARR_SOLN_MASK)|M::FLAGS_CARR_SOLN_FLOAT),Rtk::FLOAT);
  EXPECT_EQ(rtk_state(311 | M::FLAGS_REL_POS_NORM),Rtk::FIXED);
  EXPECT_EQ(rtk_state(0),Rtk::INVALID);
}
TEST(Midpoint, LocalBaselineAndDateline) {
  GeographicLib::LocalCartesian enu(35.8461944444,127.1343888889,0);
  std::array<double,3> a,b;
  enu.Reverse(0,-0.375,0,a[0],a[1],a[2]);enu.Reverse(0,0.375,0,b[0],b[1],b[2]);
  auto m=midpoint(a,b); double x,y,z; enu.Forward(m[0],m[1],m[2],x,y,z);
  EXPECT_NEAR(x,0,1e-7); EXPECT_NEAR(y,0,1e-7); EXPECT_NEAR(z,0,1e-7);
  m=midpoint({0,179.999,0},{0,-179.999,0});
  EXPECT_NEAR(std::abs(m[1]),180,1e-9);
  EXPECT_THROW(midpoint({91,0,0},{0,0,0}),std::invalid_argument);
}

TEST(Heading, LeftBaseToRightRover) {
  // Baseline compass directions for vehicle North, East, South, West.
  EXPECT_NEAR(heading_to_yaw(90,90),pi/2,1e-12);
  EXPECT_NEAR(heading_to_yaw(180,90),0,1e-12);
  EXPECT_NEAR(heading_to_yaw(270,90),-pi/2,1e-12);
  EXPECT_NEAR(std::abs(heading_to_yaw(0,90)),pi,1e-12);
}
