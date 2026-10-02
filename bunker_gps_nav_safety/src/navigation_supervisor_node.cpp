#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

class Supervisor : public rclcpp::Node {
  using Clock=std::chrono::steady_clock;
  using Time=Clock::time_point;
 public:
  Supervisor() : Node("navigation_supervisor_node") {
    rtk_timeout_=declare_parameter("rtk_timeout",1.0);
    grace_=declare_parameter("float_grace_period",3.0);
    heading_timeout_=declare_parameter("heading_timeout",4.0);
    sensor_timeout_=declare_parameter("sensor_timeout",1.0);
    cmd_timeout_=declare_parameter("cmd_timeout",0.5);
    max_speed_=declare_parameter("max_speed",0.6);
    float_speed_=declare_parameter("float_speed",0.3);
    max_yaw_=declare_parameter("max_yaw_rate",0.6);
    reverse_=declare_parameter("allow_reversing",false);
    if (!(rtk_timeout_>0 && grace_>0 && heading_timeout_>0 && sensor_timeout_>0 && cmd_timeout_>0 &&
          max_speed_>0 && max_speed_<=1.0 && float_speed_>0 && float_speed_<=max_speed_ && max_yaw_>0))
      throw std::invalid_argument("Invalid supervisor limits");
    pub_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    allowed_pub_=create_publisher<std_msgs::msg::Bool>("motion_allowed",rclcpp::QoS(1).transient_local());
    diag_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    lifecycle_=create_client<lifecycle_msgs::srv::GetState>("bt_navigator/get_state");
    nav_=rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"navigate_to_pose");
    state_sub_=create_subscription<std_msgs::msg::UInt8>("rtk_state",10,[this](std_msgs::msg::UInt8::ConstSharedPtr s) {
      auto t=Clock::now();
      if (s->data==1 && state_!=1) float_since_=t;
      state_=s->data;rtk_time_=t;
    });
    cmd_sub_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr cmd) {
      if (!std::isfinite(cmd->linear.x) || !std::isfinite(cmd->angular.z)) { cmd_=geometry_msgs::msg::Twist(); return; }
      cmd_=*cmd;cmd_time_=Clock::now();
    });
    heading_sub_=create_subscription<sensor_msgs::msg::Imu>("heading_imu",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m) {if (fresh(m->header.stamp)) heading_time_=Clock::now();});
    fix_sub_=create_subscription<sensor_msgs::msg::NavSatFix>("fix_center",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {if (fresh(m->header.stamp)) fix_time_=Clock::now();});
    local_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry_local",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m) {if (fresh(m->header.stamp)) local_time_=Clock::now();});
    gps_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry_gps",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m) {if (fresh(m->header.stamp)) gps_time_=Clock::now();});
    global_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry_global",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m) {if (fresh(m->header.stamp)) global_time_=Clock::now();});
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("imu",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m) {if (fresh(m->header.stamp)) imu_time_=Clock::now();});
    wheel_sub_=create_subscription<nav_msgs::msg::Odometry>("wheel_odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m) {if (fresh(m->header.stamp)) wheel_time_=Clock::now();});
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("obstacles",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {if (fresh(m->header.stamp)) cloud_time_=Clock::now();});
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]() {tick();});
  }
 private:
  bool fresh(const builtin_interfaces::msg::Time& stamp) {double age=(now()-rclcpp::Time(stamp)).seconds();return age>=-0.1 && age<sensor_timeout_;}
  double age(Time t) {return t==Time{} ? std::numeric_limits<double>::infinity() : std::chrono::duration<double>(Clock::now()-t).count();}
  std::string reason() {
    if (!nav_active_ || age(nav_state_time_)>3.0) return "Nav2 is not active";
    if (age(rtk_time_)>rtk_timeout_) return "RTK stream timeout";
    if (state_!=1 && state_!=2) return "RTK INVALID";
    if (state_==1 && age(float_since_)>grace_) return "RTK FLOAT timeout";
    const double gnss_limit=state_==1 ? grace_+sensor_timeout_ : sensor_timeout_;
    if (age(fix_time_)>gnss_limit || age(gps_time_)>gnss_limit) return "GNSS position timeout";
    if (age(heading_time_)>heading_timeout_) return "GNSS heading timeout";
    if (age(local_time_)>sensor_timeout_ || age(global_time_)>sensor_timeout_) return "Localization timeout";
    if (age(imu_time_)>sensor_timeout_ || age(wheel_time_)>sensor_timeout_) return "IMU/wheel timeout";
    if (age(cloud_time_)>sensor_timeout_) return "Obstacle cloud timeout";
    return "";
  }
  void tick() {
    if (state_pending_ && age(state_request_time_)>2.0) {
      lifecycle_->remove_pending_request(state_request_id_);state_pending_=false;nav_active_=false;
    }
    if (!state_pending_ && age(state_request_time_)>1.0 && lifecycle_->service_is_ready()) {
      auto future=lifecycle_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
        [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture result) {
          state_pending_=false;
          try {nav_active_=result.get()->current_state.id==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;nav_state_time_=Clock::now();}
          catch (const std::exception&) {nav_active_=false;}
        });
      state_request_id_=future.request_id;state_pending_=true;state_request_time_=Clock::now();
    }
    const auto why=reason();const bool allowed=why.empty();
    std_msgs::msg::Bool permission;permission.data=allowed;allowed_pub_->publish(permission);
    geometry_msgs::msg::Twist out;
    if (allowed && age(cmd_time_)<cmd_timeout_) {
      const double limit=state_==1 ? float_speed_ : max_speed_;
      // Scale v and omega together to preserve commanded curvature.
      const double ratio=std::max({1.0,std::abs(cmd_.linear.x)/limit,std::abs(cmd_.angular.z)/max_yaw_});
      out.linear.x=std::clamp(cmd_.linear.x/ratio,reverse_?-limit:0.0,limit);
      out.angular.z=cmd_.angular.z/ratio;
    }
    if (!allowed) {
      cmd_=geometry_msgs::msg::Twist();cmd_time_=Time{};
      if ((was_allowed_ || age(cancel_time_)>1.0) && nav_->action_server_is_ready()) {
        nav_->async_cancel_all_goals();cancel_time_=Clock::now();
      }
    }
    pub_->publish(out);
    was_allowed_=allowed;
    const auto summary=allowed ? (state_==1 ? "FLOAT: reduced speed" : "FIXED: ready") : why;
    if (summary!=previous_) {RCLCPP_INFO(get_logger(),"%s",summary.c_str());previous_=summary;}
    if (age(diag_time_)>1.0) {
      diag_time_=Clock::now();diagnostic_msgs::msg::DiagnosticArray d;d.header.stamp=now();
      diagnostic_msgs::msg::DiagnosticStatus s;s.name="navigation/supervisor";s.hardware_id="bunker";
      s.level=allowed?(state_==1?1:0):2;s.message=summary;
      diagnostic_msgs::msg::KeyValue k;k.key="nav2_action_available";k.value=nav_->action_server_is_ready()?"true":"false";s.values.push_back(k);
      d.status.push_back(s);diag_->publish(d);
    }
  }
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr lifecycle_;
  bool nav_active_{false},state_pending_{false};int64_t state_request_id_{0};
  Time nav_state_time_{},state_request_time_{};
  uint8_t state_{0};bool reverse_;bool was_allowed_{false};double rtk_timeout_,grace_,heading_timeout_,sensor_timeout_,cmd_timeout_,max_speed_,float_speed_,max_yaw_;
  Time rtk_time_{},float_since_{},heading_time_{},gps_time_{},fix_time_{},local_time_{},global_time_{},cloud_time_{},imu_time_{},wheel_time_{},cmd_time_{},cancel_time_{},diag_time_{};
  std::string previous_;geometry_msgs::msg::Twist cmd_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_sub_,imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub_,global_sub_,wheel_sub_,gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allowed_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Supervisor>());rclcpp::shutdown();}
