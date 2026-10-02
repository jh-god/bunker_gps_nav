#include <gtest/gtest.h>
#include <limits>
#include "bunker_gps_nav_perception/point_filter.hpp"
using namespace bunker_gps_nav;
TEST(PointFilter, NonFiniteBeforeGeometry) {
  const PointFilterConfig c;
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  EXPECT_EQ(classify_point(nan,0,0.2,c),PointResult::NonFinite);
  EXPECT_EQ(classify_point(0,inf,0.2,c),PointResult::NonFinite);
  EXPECT_EQ(classify_point(0,0,-inf,c),PointResult::NonFinite);
}
TEST(PointFilter, BodyIncludingAllFacesAndCorners) {
  const PointFilterConfig c;
  for (double x : {c.body_min_x,0.0,c.body_max_x})
    for (double y : {c.body_min_y,0.0,c.body_max_y})
      for (double z : {c.body_min_z,0.2,c.body_max_z})
        EXPECT_EQ(classify_point(x,y,z,c),PointResult::SelfBody);
  EXPECT_EQ(classify_point(c.body_max_x+0.001,0,0.2,c),PointResult::Keep);
  EXPECT_EQ(classify_point(c.body_min_x-0.001,0,0.2,c),PointResult::Keep);
  EXPECT_EQ(classify_point(0,c.body_max_y+0.001,0.2,c),PointResult::Keep);
  EXPECT_EQ(classify_point(0,c.body_min_y-0.001,0.2,c),PointResult::Keep);
  EXPECT_EQ(classify_point(0.5,0,c.body_max_z+0.001,c),PointResult::Keep);
}
TEST(PointFilter, OrderedHeightAndRangeWithInclusiveLimits) {
  const PointFilterConfig c;
  EXPECT_EQ(classify_point(0,0,0,c),PointResult::SelfBody); // also too low/near
  EXPECT_EQ(classify_point(60,0,2,c),PointResult::Height); // also too far
  EXPECT_EQ(classify_point(60,0,0.2,c),PointResult::Range);
  EXPECT_EQ(classify_point(0,0,1.0,c),PointResult::Range); // above body, too near
  EXPECT_EQ(classify_point(c.min_range,0,1.0,c),PointResult::Keep);
  EXPECT_EQ(classify_point(c.max_range,0,c.min_height,c),PointResult::Keep);
  EXPECT_EQ(classify_point(2,0,c.max_height,c),PointResult::Keep);
}
TEST(PointFilter, ConfigurableAsymmetricBox) {
  PointFilterConfig c;c.body_min_x=-0.3;c.body_max_x=1.0;
  EXPECT_EQ(classify_point(-0.5,0,0.2,c),PointResult::Keep);
  EXPECT_EQ(classify_point(0.9,0,0.2,c),PointResult::SelfBody);
}
