#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <robot_localization/srv/set_datum.hpp>
#include "bunker_gps_nav_localization/gnss_math.hpp"

// Explicit datum with yaw=0 makes map axes ENU in both auto and manual modes.
// Only forward measurements after SetDatum succeeds, preventing an initialization race.
class DatumManager : public rclcpp::Node {
 public:
  DatumManager() : Node("datum_manager_node") {
    mode_ = declare_parameter("datum_mode", "auto");
    if (mode_ != "auto" && mode_ != "manual")
      throw std::invalid_argument("datum_mode must be auto or manual");
    if (mode_ == "manual") {
      // Required coordinates: never fall back to a development/test location.
      datum_ = {declare_parameter("datum.latitude", rclcpp::ParameterType::PARAMETER_DOUBLE).get<double>(),
        declare_parameter("datum.longitude", rclcpp::ParameterType::PARAMETER_DOUBLE).get<double>(),
        declare_parameter("datum.altitude", 0.0)};
      if (!bunker_gps_nav::valid_lla(datum_)) throw std::invalid_argument("Invalid manual datum coordinates");
    }
    client_ = create_client<robot_localization::srv::SetDatum>("datum");
    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix_navsat", 10);
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("fix_center", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr f) {
        if (ready_) { pub_->publish(*f); return; }
        if (mode_ == "auto" && !have_datum_) { datum_ = {f->latitude,f->longitude,f->altitude}; have_datum_=true; }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (ready_ || pending_ || (mode_=="auto" && !have_datum_) || !client_->service_is_ready()) return;
      auto req = std::make_shared<robot_localization::srv::SetDatum::Request>();
      req->geo_pose.position.latitude=datum_[0]; req->geo_pose.position.longitude=datum_[1]; req->geo_pose.position.altitude=datum_[2];
      req->geo_pose.orientation.w=1.0;
      pending_=true;
      client_->async_send_request(req, [this](rclcpp::Client<robot_localization::srv::SetDatum>::SharedFuture f) {
        try { f.get(); ready_=true; RCLCPP_INFO(get_logger(), "ENU datum: %.9f %.9f %.3f",datum_[0],datum_[1],datum_[2]); }
        catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "%s", e.what()); }
        pending_=false;
      });
    });
  }
 private:
  std::string mode_; std::array<double,3> datum_{};
  bool ready_{false}, pending_{false}, have_datum_{false};
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<DatumManager>());rclcpp::shutdown();}
