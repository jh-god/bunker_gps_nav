#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "bunker_gps_nav_localization/gnss_math.hpp"

class HeadingNode : public rclcpp::Node {
 public:
  HeadingNode() : Node("gnss_heading_node") {
    offset_ = declare_parameter("heading_mount_offset_deg", 90.0);
    min_accuracy_ = declare_parameter("min_heading_accuracy_deg", 0.1);
    max_accuracy_ = declare_parameter("max_heading_accuracy_deg", 5.0);
    frame_ = declare_parameter("heading_frame", "base_link");
    if (min_accuracy_ <= 0 || max_accuracy_ < min_accuracy_ || !std::isfinite(offset_))
      throw std::invalid_argument("Invalid heading configuration");
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("heading_imu", 10);
    state_pub_ = create_publisher<std_msgs::msg::UInt8>("rtk_state", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    sub_ = create_subscription<ublox_msgs::msg::NavRELPOSNED9>("relposned", rclcpp::SensorDataQoS(),
      [this](ublox_msgs::msg::NavRELPOSNED9::ConstSharedPtr m) {
        // This message has no Header. Stamp at receipt; reject repeated receiver epochs.
        if (have_epoch_ && m->i_tow == last_epoch_) return;
        have_epoch_ = true; last_epoch_ = m->i_tow;
        auto state = bunker_gps_nav::rtk_state(m->flags);
        auto accuracy = m->acc_heading * 1e-5;
        const bool valid = bunker_gps_nav::heading_valid(m->flags) && accuracy <= max_accuracy_;
        std_msgs::msg::UInt8 s; s.data = static_cast<uint8_t>(state); state_pub_->publish(s);
        if (valid && state == bunker_gps_nav::Rtk::FIXED) {
          sensor_msgs::msg::Imu imu;
          imu.header.stamp = now(); imu.header.frame_id = frame_;
          tf2::Quaternion q;
          q.setRPY(0, 0, bunker_gps_nav::heading_to_yaw(m->rel_pos_heading * 1e-5, offset_));
          imu.orientation = tf2::toMsg(q);
          imu.orientation_covariance[0] = imu.orientation_covariance[4] = 1e6;
          imu.orientation_covariance[8] = std::pow(std::max(accuracy, min_accuracy_) * bunker_gps_nav::pi / 180, 2);
          imu.angular_velocity_covariance[0] = -1;
          imu.linear_acceleration_covariance[0] = -1;
          imu_pub_->publish(imu);
        }
        const std::string label = state == bunker_gps_nav::Rtk::FIXED ? "FIXED" :
          state == bunker_gps_nav::Rtk::FLOAT ? "FLOAT" : "INVALID";
        auto summary = label + (valid ? ": heading valid" : ": heading invalid");
        if (summary != previous_) { RCLCPP_INFO(get_logger(), "%s", summary.c_str()); previous_ = summary; }
        diagnostic_msgs::msg::DiagnosticArray d; d.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus ds;
        ds.name = "gnss/heading"; ds.hardware_id = "dual_gnss";
        ds.level = state == bunker_gps_nav::Rtk::FIXED && valid ? 0 : 1; ds.message = summary;
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "accuracy_deg"; kv.value = std::to_string(accuracy); ds.values.push_back(kv);
        kv.key = "flags"; kv.value = std::to_string(m->flags); ds.values.push_back(kv);
        d.status.push_back(ds); diag_pub_->publish(d);
      });
  }
 private:
  double offset_, min_accuracy_, max_accuracy_;
  std::string frame_, previous_;
  uint32_t last_epoch_{0}; bool have_epoch_{false};
  rclcpp::Subscription<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};
int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<HeadingNode>()); rclcpp::shutdown(); }
