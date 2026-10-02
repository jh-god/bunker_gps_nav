#include <chrono>
#include <array>
#include <algorithm>
#include "bunker_gps_nav_perception/point_filter.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

class ObstacleFilter : public rclcpp::Node {
 public:
  ObstacleFilter() : Node("obstacle_cloud_filter_node"), buffer_(get_clock()), listener_(buffer_) {
    frame_ = declare_parameter("output_frame", "base_link");
    filter_.min_height = declare_parameter("min_obstacle_height", 0.05);
    filter_.max_height = declare_parameter("max_obstacle_height", 1.5);
    filter_.min_range = declare_parameter("min_range", 0.3);
    filter_.max_range = declare_parameter("max_range", 50.0);
    filter_.body_min_x = declare_parameter("self_body_min_x", -0.65);
    filter_.body_max_x = declare_parameter("self_body_max_x", 0.65);
    filter_.body_min_y = declare_parameter("self_body_min_y", -0.45);
    filter_.body_max_y = declare_parameter("self_body_max_y", 0.45);
    filter_.body_min_z = declare_parameter("self_body_min_z", -0.30);
    filter_.body_max_z = declare_parameter("self_body_max_z", 0.60);
    voxel_ = declare_parameter("use_voxel_filter", true);
    leaf_ = declare_parameter("voxel_leaf_size", 0.1);
    timeout_ = declare_parameter("transform_timeout", 0.1);
    max_age_ = declare_parameter("max_cloud_age", 1.0);
    if (!(filter_.max_height > filter_.min_height && filter_.max_range > filter_.min_range && filter_.min_range >= 0 && leaf_ > 0 && timeout_>=0 && max_age_>0))
      throw std::invalid_argument("Invalid cloud filter parameters");
    const std::array<double,13> values = {filter_.min_height,filter_.max_height,
      filter_.min_range,filter_.max_range,filter_.body_min_x,filter_.body_max_x,
      filter_.body_min_y,filter_.body_max_y,filter_.body_min_z,filter_.body_max_z,
      leaf_,timeout_,max_age_};
    if (!std::all_of(values.begin(),values.end(),[](double v) {return std::isfinite(v);}) ||
        filter_.body_min_x >= filter_.body_max_x || filter_.body_min_y >= filter_.body_max_y ||
        filter_.body_min_z >= filter_.body_max_z) throw std::invalid_argument("Invalid self-body box or non-finite filter limits");
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacles", rclcpp::SensorDataQoS());
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("nonground", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { process(*msg); });
  }
 private:
  void process(const sensor_msgs::msg::PointCloud2& msg) {
    const auto start = std::chrono::steady_clock::now();
    try {
      const auto age = (now()-rclcpp::Time(msg.header.stamp)).seconds();
      if (age > max_age_ || age < -0.1 || msg.header.frame_id.empty()) return;
      // Validate layout before handing bytes to PCL (empty clouds are valid).
      bool x=false,y=false,z=false;
      for (const auto& f:msg.fields) {
        if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count!=1 || f.offset+4>msg.point_step) continue;
        x |= f.name=="x"; y |= f.name=="y"; z |= f.name=="z";
      }
      if (!x || !y || !z || msg.is_bigendian || msg.row_step < uint64_t(msg.width)*msg.point_step ||
          msg.data.size() < uint64_t(msg.row_step)*msg.height) throw std::runtime_error("Invalid XYZ PointCloud2 layout");
      sensor_msgs::msg::PointCloud2 transformed;
      if (msg.header.frame_id == frame_) transformed = msg;
      else tf2::doTransform(msg, transformed, buffer_.lookupTransform(frame_,msg.header.frame_id,
        rclcpp::Time(msg.header.stamp), rclcpp::Duration::from_seconds(timeout_)));
      pcl::PointCloud<pcl::PointXYZ> input; pcl::fromROSMsg(transformed,input);
      auto filtered=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      filtered->reserve(input.size());
      std::array<size_t,5> counts{};
      // Non-finite -> self body -> height -> range; voxel downsampling follows.
      for (const auto& p : input) {
        const auto result = bunker_gps_nav::classify_point(p.x,p.y,p.z,filter_);
        ++counts[static_cast<size_t>(result)];
        if (result == bunker_gps_nav::PointResult::Keep) filtered->push_back(p);
      }
      pcl::PointCloud<pcl::PointXYZ> downsampled;
      if (voxel_ && !filtered->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> v; v.setInputCloud(filtered); v.setLeafSize(leaf_,leaf_,leaf_); v.filter(downsampled);
      } else downsampled=*filtered;
      sensor_msgs::msg::PointCloud2 output; pcl::toROSMsg(downsampled,output);
      output.header=msg.header; output.header.frame_id=frame_; pub_->publish(output);
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      const double dt=std::chrono::duration<double>(start-last_cloud_).count(); last_cloud_=start;
      if (std::chrono::duration<double>(start-last_diag_).count() < 1.0) return;
      last_diag_=start;
      diagnostic_msgs::msg::DiagnosticArray d; d.header.stamp=now();
      diagnostic_msgs::msg::DiagnosticStatus s; s.name="perception/obstacle_filter"; s.hardware_id="lidar";
      s.level=0; s.message="Cloud filtered in base frame";
      auto add=[&s](const std::string& k, double val) {diagnostic_msgs::msg::KeyValue v;v.key=k;v.value=std::to_string(val);s.values.push_back(v);};
      add("input_points",input.size());add("output_points",downsampled.size());add("processing_ms",ms);add("frequency_hz",dt>0?1/dt:0);
      add("non_finite_removed",counts[1]);add("self_body_removed",counts[2]);
      add("height_removed",counts[3]);add("range_removed",counts[4]);
      d.status.push_back(s);diag_->publish(d);
    } catch (const std::exception& e) { RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Cloud rejected: %s",e.what()); }
  }
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  bunker_gps_nav::PointFilterConfig filter_;
  std::string frame_;double leaf_,timeout_,max_age_;bool voxel_;
  std::chrono::steady_clock::time_point last_diag_{},last_cloud_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ObstacleFilter>());rclcpp::shutdown();}
