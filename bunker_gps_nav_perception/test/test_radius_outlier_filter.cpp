#include <gtest/gtest.h>
#include <limits>
#include <pcl/filters/voxel_grid.h>
#include "bunker_gps_nav_perception/radius_outlier_filter.hpp"

using namespace bunker_gps_nav;

namespace {
RadiusOutlierConfig enabled() {
  RadiusOutlierConfig c;
  c.enabled = true;
  return c;
}
ObstacleCloud::Ptr cloud(std::initializer_list<pcl::PointXYZ> points) {
  auto input = pcl::make_shared<ObstacleCloud>();
  for (const auto &p : points) input->push_back(p);
  return input;
}
}  // namespace

TEST(RadiusOutlierFilter, RemovesNearSingletonPreservesPairAndDistantSingleton) {
  const auto input = cloud({{1.0f,1.0f,0.2f}, {1.5f,1.0f,0.2f},
                           {1.6f,1.0f,0.2f}, {5.0f,1.0f,0.2f}});
  const auto output = remove_near_outliers(input, enabled());
  ASSERT_EQ(output->size(),3u);
  EXPECT_FLOAT_EQ((*output)[0].x,1.5f);
  EXPECT_FLOAT_EQ((*output)[1].x,1.6f);
  EXPECT_FLOAT_EQ((*output)[2].x,5.0f);
  EXPECT_EQ(input->size(),4u);
}

TEST(RadiusOutlierFilter, UsesSupportAcrossRangeBoundary) {
  const auto input = cloud({{2.95f,0.0f,0.2f}, {3.05f,0.0f,0.2f}});
  EXPECT_EQ(remove_near_outliers(input,enabled())->size(),2u);
}

TEST(RadiusOutlierFilter, RangeIsBaseXYAndRadiusIsThreeDimensional) {
  const auto input = cloud({{3.0f,0.0f,1.0f}, {3.001f,1.0f,1.0f},
                           {1.0f,0.0f,0.0f}, {1.0f,0.0f,0.4f}});
  const auto output = remove_near_outliers(input,enabled());
  ASSERT_EQ(output->size(),1u);
  EXPECT_FLOAT_EQ((*output)[0].x,3.001f);
}

TEST(RadiusOutlierFilter, NeighborCountExcludesQueryPoint) {
  const auto single = cloud({{1.0f,0.0f,0.2f}});
  EXPECT_TRUE(remove_near_outliers(single,enabled())->empty());
  const auto pair = cloud({{1.0f,0.0f,0.2f}, {1.05f,0.0f,0.2f}});
  EXPECT_EQ(remove_near_outliers(pair,enabled())->size(),2u);
  auto c = enabled();
  c.min_neighbors = 2;
  EXPECT_TRUE(remove_near_outliers(pair,c)->empty());
  pair->push_back({1.1f,0.0f,0.2f});
  EXPECT_EQ(remove_near_outliers(pair,c)->size(),3u);
}

TEST(RadiusOutlierFilter, PreservesCloudWhenDisabledOrNoNearPoints) {
  const auto input = cloud({{5.0f,0.0f,0.2f}});
  EXPECT_EQ(remove_near_outliers(input,enabled()),input);
  EXPECT_EQ(remove_near_outliers(input,RadiusOutlierConfig()),input);
  const auto empty = cloud({});
  EXPECT_EQ(remove_near_outliers(empty,enabled()),empty);
}

TEST(RadiusOutlierFilter, DenoisesBeforeVoxelCollapsesRealObjectHits) {
  const auto input = cloud({{1.01f,0.01f,0.21f}, {1.02f,0.01f,0.21f},
                           {2.0f,0.0f,0.2f}});
  const auto denoised = remove_near_outliers(input,enabled());
  ASSERT_EQ(denoised->size(),2u);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(denoised);
  voxel.setLeafSize(0.1f,0.1f,0.1f);
  ObstacleCloud downsampled;
  voxel.filter(downsampled);
  ASSERT_EQ(downsampled.size(),1u);
  EXPECT_NEAR(downsampled[0].x,1.015f,1e-5);
}

TEST(RadiusOutlierFilter, RejectsInvalidConfiguration) {
  auto c = enabled();
  EXPECT_TRUE(valid_radius_outlier_config(c));
  c.radius = 0;
  EXPECT_FALSE(valid_radius_outlier_config(c));
  c.radius = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_radius_outlier_config(c));
  c = enabled(); c.max_range = -1;
  EXPECT_FALSE(valid_radius_outlier_config(c));
  c = enabled(); c.max_range = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(valid_radius_outlier_config(c));
  c = enabled(); c.min_neighbors = 0;
  EXPECT_FALSE(valid_radius_outlier_config(c));
}
