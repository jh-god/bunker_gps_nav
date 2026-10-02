#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "bunker_gps_nav_localization/gnss_math.hpp"

class MidpointNode : public rclcpp::Node {
  using Fix = sensor_msgs::msg::NavSatFix;
 public:
  MidpointNode() : Node("gnss_midpoint_node") {
    delta_ = declare_parameter("max_fix_time_difference", 0.15);
    max_age_ = declare_parameter("max_fix_age", 1.0);
    floor_ = declare_parameter("position_variance_floor", 0.0004);
    unknown_ = declare_parameter("unknown_position_variance", 4.0);
    frame_ = declare_parameter("center_frame", "base_link");
    if (delta_ <= 0 || max_age_ <= 0 || floor_ <= 0 || unknown_ < floor_)
      throw std::invalid_argument("Invalid midpoint thresholds");
    pub_ = create_publisher<Fix>("fix_center", 10);
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    rtk_ = create_subscription<std_msgs::msg::UInt8>("rtk_state", 10, [this](std_msgs::msg::UInt8::ConstSharedPtr m) {
      state_ = m->data; state_time_ = now(); try_publish();
    });
    left_ = create_subscription<Fix>("base_fix", rclcpp::SensorDataQoS(), [this](Fix::ConstSharedPtr m) { a_ = m; try_publish(); });
    right_ = create_subscription<Fix>("rover_fix", rclcpp::SensorDataQoS(), [this](Fix::ConstSharedPtr m) { b_ = m; try_publish(); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      diagnostic_msgs::msg::DiagnosticArray d; d.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus s; s.name = "gnss/midpoint"; s.hardware_id = "dual_gnss";
      s.level = (now() - published_).seconds() < max_age_ ? 0 : 2;
      s.message = s.level == 0 ? "Synchronized FIXED midpoint" : "Waiting for fresh synchronized FIXED fixes";
      d.status.push_back(s); diag_->publish(d);
    });
  }
 private:
  bool valid(const Fix& f) {
    const auto age = (now() - rclcpp::Time(f.header.stamp)).seconds();
    return f.status.status >= 0 && age >= -0.1 && age <= max_age_ &&
      bunker_gps_nav::valid_lla({f.latitude, f.longitude, f.altitude});
  }
  void try_publish() {
    if (!a_ || !b_ || state_ != 2 || (now()-state_time_).seconds() > max_age_) return;
    if (!valid(*a_) || !valid(*b_)) return;
    if (std::abs((rclcpp::Time(a_->header.stamp)-rclcpp::Time(b_->header.stamp)).seconds()) > delta_) return;
    auto p = bunker_gps_nav::midpoint({a_->latitude,a_->longitude,a_->altitude}, {b_->latitude,b_->longitude,b_->altitude});
    Fix out; out.header.frame_id = frame_;
    out.header.stamp = rclcpp::Time((rclcpp::Time(a_->header.stamp).nanoseconds()+rclcpp::Time(b_->header.stamp).nanoseconds())/2);
    out.latitude = p[0]; out.longitude = p[1]; out.altitude = p[2];
    out.status.status = std::min(a_->status.status,b_->status.status);
    out.status.service = a_->status.service | b_->status.service;
    // Shared RTK corrections correlate the two receivers. Do not claim sqrt(2) improvement.
    for (int i : {0,4,8}) {
      auto variance = [this,i](const Fix& f) {
        const auto v=f.position_covariance[i];
        return f.position_covariance_type == Fix::COVARIANCE_TYPE_UNKNOWN || !std::isfinite(v) || v<=0 ? unknown_ : v;
      };
      out.position_covariance[i] = std::max({floor_, variance(*a_), variance(*b_)});
    }
    out.position_covariance_type = Fix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    pub_->publish(out); published_ = now(); a_.reset(); b_.reset();
  }
  double delta_, max_age_, floor_, unknown_; std::string frame_;
  uint8_t state_{0};
  rclcpp::Time state_time_{0,0,RCL_ROS_TIME}, published_{0,0,RCL_ROS_TIME};
  Fix::ConstSharedPtr a_, b_;
  rclcpp::Subscription<Fix>::SharedPtr left_, right_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr rtk_;
  rclcpp::Publisher<Fix>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<MidpointNode>()); rclcpp::shutdown(); }
