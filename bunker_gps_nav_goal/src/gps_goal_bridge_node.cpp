#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GoalBridge : public rclcpp::Node {
  using Nav=nav2_msgs::action::NavigateToPose;
  using Clock=std::chrono::steady_clock;
 public:
  GoalBridge() : Node("gps_goal_bridge_node"), buffer_(get_clock()),listener_(buffer_) {
    frame_=declare_parameter("map_frame","map");
    max_distance_=declare_parameter("max_goal_distance",25.0);
    if (max_distance_<=0) throw std::invalid_argument("max_goal_distance must be positive");
    nav_=rclcpp_action::create_client<Nav>(this,"navigate_to_pose");
    converter_=create_client<robot_localization::srv::FromLL>("fromLL");
    allowed_sub_=create_subscription<std_msgs::msg::Bool>("motion_allowed",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {allowed_=m->data;permission_time_=Clock::now();});
    gps_odom_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry_gps",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {if ((now()-rclcpp::Time(m->header.stamp)).seconds()<1) gps_time_=Clock::now();});
    gps_sub_=create_subscription<geographic_msgs::msg::GeoPoseStamped>("gps_goal",10,
      [this](geographic_msgs::msg::GeoPoseStamped::ConstSharedPtr m) {gps(*m);});
    rviz_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {++sequence_;send(*m);});
    timeout_timer_=create_wall_timer(std::chrono::milliseconds(200),[this]() {
      if (request_pending_ && age(request_time_)>2) {
        converter_->remove_pending_request(request_id_);request_pending_=false;++sequence_;
        RCLCPP_WARN(get_logger(),"Coordinate conversion timed out; resend goal when localization is ready");
      }
    });
  }
 private:
  double age(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
  bool ready() {return allowed_ && age(permission_time_)<1.0 && nav_->action_server_is_ready();}
  static bool normalize(geometry_msgs::msg::Quaternion& q) {
    double norm=std::hypot(std::hypot(q.x,q.y),std::hypot(q.z,q.w));
    if (!std::isfinite(norm) || norm<1e-6) return false;
    q.x/=norm;q.y/=norm;q.z/=norm;q.w/=norm;return true;
  }
  void gps(const geographic_msgs::msg::GeoPoseStamped& m) {
    const auto& p=m.pose.position;
    auto orientation=m.pose.orientation;
    if (!std::isfinite(p.latitude) || !std::isfinite(p.longitude) || !std::isfinite(p.altitude) ||
        std::abs(p.latitude)>90 || std::abs(p.longitude)>180 || !normalize(orientation)) {
      RCLCPP_ERROR(get_logger(),"Invalid GPS goal coordinates/quaternion");return;
    }
    if (!ready() || age(gps_time_)>1 || !converter_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),"GPS goal rejected: localization, safety gate or Nav2 is not ready");return;
    }
    if (request_pending_) {converter_->remove_pending_request(request_id_);request_pending_=false;}
    const auto sequence=++sequence_;
    auto req=std::make_shared<robot_localization::srv::FromLL::Request>();req->ll_point=p;
    auto result=converter_->async_send_request(req,[this,orientation,sequence](rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture f) {
      if (sequence!=sequence_) return;
      request_pending_=false;
      try {
        geometry_msgs::msg::PoseStamped goal;goal.header.frame_id=frame_;goal.header.stamp=now();
        goal.pose.position=f.get()->map_point;goal.pose.position.z=0;goal.pose.orientation=orientation;
        send(goal);
      } catch (const std::exception& e) {RCLCPP_ERROR(get_logger(),"GPS conversion failed: %s",e.what());}
    });
    request_id_=result.request_id;request_pending_=true;request_time_=Clock::now();
  }
  void send(geometry_msgs::msg::PoseStamped goal) {
    if (!ready()) {RCLCPP_WARN(get_logger(),"Goal rejected: navigation is not ready");return;}
    if (!normalize(goal.pose.orientation) || !std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y)) return;
    try {
      if (goal.header.frame_id!=frame_) goal=buffer_.transform(goal,frame_,tf2::durationFromSec(0.1));
      auto robot=buffer_.lookupTransform(frame_,base_frame(),tf2::TimePointZero);
      const double distance=std::hypot(goal.pose.position.x-robot.transform.translation.x,goal.pose.position.y-robot.transform.translation.y);
      if (distance>max_distance_) {RCLCPP_WARN(get_logger(),"Goal %.1fm away exceeds rolling costmap limit %.1fm",distance,max_distance_);return;}
    } catch (const tf2::TransformException& e) {RCLCPP_WARN(get_logger(),"Goal TF unavailable: %s",e.what());return;}
    goal.header.stamp=now();Nav::Goal request;request.pose=goal;
    auto options=rclcpp_action::Client<Nav>::SendGoalOptions();
    options.goal_response_callback=[this](rclcpp_action::ClientGoalHandle<Nav>::SharedPtr h) {
      RCLCPP_INFO(get_logger(),"Nav2 goal %s",h?"accepted":"rejected");
    };
    options.result_callback=[this](const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult& r) {
      RCLCPP_INFO(get_logger(),"Nav2 goal result: %s",r.code==rclcpp_action::ResultCode::SUCCEEDED?"reached":r.code==rclcpp_action::ResultCode::CANCELED?"canceled":"aborted");
    };
    nav_->async_send_goal(request,options);
  }
  std::string base_frame() {if (!has_parameter("base_frame")) return declare_parameter("base_frame","base_link");return get_parameter("base_frame").as_string();}
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  std::string frame_;double max_distance_;bool allowed_{false},request_pending_{false};int64_t request_id_{0};uint64_t sequence_{0};
  Clock::time_point gps_time_{},permission_time_{},request_time_{};
  rclcpp_action::Client<Nav>::SharedPtr nav_;
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr converter_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allowed_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<GoalBridge>());rclcpp::shutdown();}
