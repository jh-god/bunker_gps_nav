#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// State is accessed on this node's single-threaded executor. Every waypoint uses
// the same guarded NavigateToPose path as an ordinary RViz/GPS goal.
class GoalBridge : public rclcpp::Node {
  using Nav = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ClientGoalHandle<Nav>;
  using Trigger = std_srvs::srv::Trigger;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Clock = std::chrono::steady_clock;
 public:
  GoalBridge() : Node("gps_goal_bridge_node"), buffer_(get_clock()), listener_(buffer_) {
    frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_link");
    max_distance_ = declare_parameter("max_goal_distance", 25.0);
    max_waypoints_ = declare_parameter("max_waypoints", 200);
    permission_timeout_ = declare_parameter("permission_timeout", 1.0);
    if (!std::isfinite(max_distance_) || max_distance_ <= 0 || max_waypoints_ <= 0 ||
        !std::isfinite(permission_timeout_) || permission_timeout_ <= 0) {
      throw std::invalid_argument("Goal distance, waypoint count and permission timeout must be positive");
    }
    nav_ = rclcpp_action::create_client<Nav>(this, "navigate_to_pose");
    converter_ = create_client<robot_localization::srv::FromLL>("fromLL");
    auto persistent = rclcpp::QoS(1).transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>("waypoints/status", persistent);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints/markers", persistent);
    allowed_sub_ = create_subscription<std_msgs::msg::Bool>("motion_allowed", persistent,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        allowed_ = m->data;
        permission_time_ = Clock::now();
        if (!allowed_ && active_) {
          replacement_.reset();
          if (!cancel_requested_) stop("Paused: motion permission withdrawn");
        }
      });
    gps_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_gps", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const double dt = (now() - rclcpp::Time(m->header.stamp)).seconds();
        if (dt >= -0.1 && dt < 1.0) gps_time_ = Clock::now();
      });
    gps_sub_ = create_subscription<geographic_msgs::msg::GeoPoseStamped>("gps_goal", 10,
      [this](geographic_msgs::msg::GeoPoseStamped::ConstSharedPtr m) { gps(*m); });
    rviz_sub_ = create_subscription<Pose>("goal_pose", 10,
      [this](Pose::ConstSharedPtr m) { invalidate_conversion(); single(*m); });
    waypoint_sub_ = create_subscription<Pose>("waypoint", 10,
      [this](Pose::ConstSharedPtr m) { add_waypoint(*m); });
    start_srv_ = create_service<Trigger>("waypoints/start",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) { start(*r); });
    cancel_srv_ = create_service<Trigger>("waypoints/cancel",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
        invalidate_conversion();
        replacement_.reset();
        stop("Paused: canceled by operator");
        r->success = true;
        r->message = active_ ? "Cancellation requested; waiting for Nav2" : "Stopped";
      });
    clear_srv_ = create_service<Trigger>("waypoints/clear",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
        if (active_ || request_pending_) { r->message = "Cancel and wait for Nav2 before editing"; return; }
        waypoints_.clear();
        next_ = 0;
        r->success = true;
        r->message = "Waypoints cleared";
        update("Ready");
      });
    undo_srv_ = create_service<Trigger>("waypoints/remove_last",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
        if (active_ || request_pending_) { r->message = "Cancel and wait for Nav2 before editing"; return; }
        if (waypoints_.empty()) { r->message = "No waypoints to remove"; return; }
        waypoints_.pop_back();
        next_ = std::min(next_, waypoints_.size());
        r->success = true;
        r->message = "Last waypoint removed";
        update("Ready");
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (request_pending_ && age(request_time_) > 2.0) {
        invalidate_conversion();
        update("GPS coordinate conversion timed out");
      }
      if (active_ && !ready()) {
        replacement_.reset();
        if (!cancel_requested_) stop("Paused: safety gate or Nav2 unavailable");
      } else if (active_ && !cancel_requested_ && !handle_ && age(sent_time_) > 5.0) {
        stop("Paused: Nav2 goal acknowledgment timed out");
      }
      // Stops during acknowledgment still cancel a subsequently accepted goal.
      // Remain busy until rejection/result arrives, preventing overlapping goals.
      if (active_ && cancel_requested_ && handle_ && !cancel_inflight_ && age(cancel_time_) > 1.0) {
        cancel_active();
      }
    });
    update("Ready");
  }
 private:
  static double age(Clock::time_point t) {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }
  bool ready() const {
    return allowed_ && age(permission_time_) < permission_timeout_ && nav_->action_server_is_ready();
  }
  static bool normalize(geometry_msgs::msg::Quaternion &q) {
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(norm) || norm < 1e-6) return false;
    q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
    return true;
  }
  bool in_map(Pose &goal, std::string &error) {
    const auto &p = goal.pose.position;
    if (goal.header.frame_id.empty() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z) || !normalize(goal.pose.orientation)) {
      error = "Invalid goal frame, position or quaternion";
      return false;
    }
    try {
      if (goal.header.frame_id != frame_) goal = buffer_.transform(goal, frame_, tf2::durationFromSec(0.1));
      goal.pose.position.z = 0;  // Nav2 goals lie in the map XY plane.
      return true;
    } catch (const tf2::TransformException &e) {
      error = std::string("Goal TF unavailable: ") + e.what();
      return false;
    }
  }
  bool within_robot_limit(const Pose &goal, std::string &error) {
    try {
      const auto robot = buffer_.lookupTransform(frame_, base_frame_, tf2::TimePointZero);
      const double distance = std::hypot(goal.pose.position.x - robot.transform.translation.x,
                                        goal.pose.position.y - robot.transform.translation.y);
      if (!std::isfinite(distance) || distance > max_distance_) {
        error = "Goal exceeds robot-to-goal distance limit (" + std::to_string(max_distance_) + " m)";
        return false;
      }
      return true;
    } catch (const tf2::TransformException &e) {
      error = std::string("Robot TF unavailable: ") + e.what();
      return false;
    }
  }
  void invalidate_conversion() {
    ++sequence_;
    if (request_pending_) converter_->remove_pending_request(request_id_);
    request_pending_ = false;
  }
  bool waypoint_busy() const { return running_ || (active_ && waypoint_goal_); }
  void gps(const geographic_msgs::msg::GeoPoseStamped &m) {
    auto orientation = m.pose.orientation;
    const auto &p = m.pose.position;
    if (waypoint_busy()) { RCLCPP_WARN(get_logger(), "GPS goal ignored during waypoint execution"); return; }
    if (!std::isfinite(p.latitude) || !std::isfinite(p.longitude) || !std::isfinite(p.altitude) ||
        std::abs(p.latitude) > 90 || std::abs(p.longitude) > 180 || !normalize(orientation)) {
      update("Invalid GPS goal coordinates/quaternion"); return;
    }
    if (!ready() || age(gps_time_) > 1.0 || !converter_->service_is_ready()) {
      update("GPS goal rejected: localization, safety gate or Nav2 is not ready"); return;
    }
    invalidate_conversion();
    const auto sequence = sequence_;
    auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
    req->ll_point = p;
    auto result = converter_->async_send_request(req,
      [this, orientation, sequence](rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture f) {
        if (sequence != sequence_) return;
        request_pending_ = false;
        try {
          Pose goal;
          goal.header.frame_id = frame_;
          goal.header.stamp = now();
          goal.pose.position = f.get()->map_point;
          goal.pose.position.z = 0;
          goal.pose.orientation = orientation;
          single(goal);
        } catch (const std::exception &e) { update(std::string("GPS conversion failed: ") + e.what()); }
      });
    request_id_ = result.request_id;
    request_pending_ = true;
    request_time_ = Clock::now();
  }
  void single(Pose goal) {
    if (waypoint_busy()) { RCLCPP_WARN(get_logger(), "Single goal ignored during waypoint execution"); return; }
    std::string error;
    if (!ready()) { update("Goal rejected: navigation is not ready"); return; }
    if (!in_map(goal, error) || !within_robot_limit(goal, error)) { update(error); return; }
    if (active_) {
      replacement_ = goal;
      stop("Replacing single goal");
    } else {
      dispatch(goal, false);
    }
  }
  void add_waypoint(Pose goal) {
    std::string error;
    if (active_ || request_pending_) { RCLCPP_WARN(get_logger(), "Cancel and wait before editing waypoints"); return; }
    if (waypoints_.size() >= static_cast<size_t>(max_waypoints_)) { update("Waypoint count limit reached"); return; }
    if (!in_map(goal, error)) { update(error); return; }
    waypoints_.push_back(goal);
    update("Ready");
  }
  void start(Trigger::Response &r) {
    if (active_ || request_pending_) { r.message = "Navigation is busy; wait for the current goal/cancellation"; return; }
    if (next_ >= waypoints_.size()) { r.message = "No pending waypoints; add points or clear the completed route"; return; }
    if (!ready()) { r.message = "Localization, safety gate or Nav2 is not ready"; return; }
    if (!within_robot_limit(waypoints_[next_], r.message)) return;
    for (size_t i = next_ + 1; i < waypoints_.size(); ++i) {
      const auto &a = waypoints_[i - 1].pose.position;
      const auto &b = waypoints_[i].pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) > max_distance_) {
        r.message = "Segment " + std::to_string(i) + " -> " + std::to_string(i + 1) +
                    " exceeds distance limit; undo or clear and add closer waypoints";
        return;
      }
    }
    running_ = true;
    dispatch_waypoint();
    r.success = running_;
    r.message = running_ ? "Waypoint execution started" : state_;
  }
  void dispatch_waypoint() {
    std::string error;
    if (!ready()) { running_ = false; update("Paused: navigation is not ready"); return; }
    if (!within_robot_limit(waypoints_[next_], error)) { running_ = false; update("Paused: " + error); return; }
    dispatch(waypoints_[next_], true);
  }
  void dispatch(Pose goal, bool waypoint) {
    goal.header.stamp = now();
    Nav::Goal request;
    request.pose = goal;
    active_ = true;
    waypoint_goal_ = waypoint;
    cancel_requested_ = false;
    cancel_inflight_ = false;
    handle_.reset();
    sent_time_ = Clock::now();
    const auto token = ++goal_token_;
    auto options = rclcpp_action::Client<Nav>::SendGoalOptions();
    options.goal_response_callback = [this, token](Handle::SharedPtr h) {
      if (token != goal_token_) return;
      if (!h) { finish(rclcpp_action::ResultCode::ABORTED, "Nav2 rejected goal"); return; }
      handle_ = h;
      if (cancel_requested_) cancel_active();
      else if (!ready()) stop("Paused: cancellation pending");
    };
    options.result_callback = [this, token](const Handle::WrappedResult &r) {
      if (token == goal_token_) finish(r.code, "Nav2 goal failed or was canceled");
    };
    try {
      nav_->async_send_goal(request, options);
      update(waypoint ? "Running" : "Single goal running");
    } catch (const std::exception &e) { finish(rclcpp_action::ResultCode::ABORTED, e.what()); }
  }
  void cancel_active() {
    if (!handle_ || cancel_inflight_) return;
    cancel_time_ = Clock::now();
    cancel_inflight_ = true;
    const auto token = goal_token_;
    try {
      nav_->async_cancel_goal(handle_, [this, token](rclcpp_action::Client<Nav>::CancelResponse::SharedPtr) {
        if (token == goal_token_) cancel_inflight_ = false;
      });
    } catch (const std::exception &e) {
      cancel_inflight_ = false;
      RCLCPP_WARN(get_logger(), "Nav2 cancellation: %s", e.what());
    }
  }
  void stop(const std::string &reason) {
    running_ = false;
    if (active_) {
      cancel_requested_ = true;
      cancel_active();
    }
    update(reason);
  }
  void finish(rclcpp_action::ResultCode code, const std::string &error) {
    const bool stopped = cancel_requested_;
    const bool was_waypoint = waypoint_goal_;
    active_ = false;
    handle_.reset();
    cancel_inflight_ = false;
    // Even SUCCEEDED can arrive after a stop: do not advance or resume then.
    if (was_waypoint && !stopped && code == rclcpp_action::ResultCode::SUCCEEDED) {
      ++next_;
      if (next_ < waypoints_.size()) { dispatch_waypoint(); return; }
      running_ = false;
      update("Completed");
    } else if (!stopped) {
      running_ = false;
      update(code == rclcpp_action::ResultCode::SUCCEEDED ? "Single goal reached" : "Paused: " + error);
    } else {
      update(state_);
    }
    if (replacement_) {
      auto goal = *replacement_;
      replacement_.reset();
      single(goal);  // Recheck readiness and actual robot distance after cancel.
    }
  }
  void update(const std::string &state) {
    state_ = state;
    std::ostringstream text;
    text << state_ << "\nReached " << next_ << " / " << waypoints_.size();
    if (next_ < waypoints_.size()) text << " | Next: " << next_ + 1;
    if (active_ && cancel_requested_) text << "\nWaiting for Nav2 cancellation";
    std_msgs::msg::String status;
    status.data = text.str();
    status_pub_->publish(status);
    RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
    publish_markers();
  }
  void publish_markers() {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear;
    clear.header.frame_id = frame_;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);
    Marker line;
    line.header.frame_id = frame_;
    line.ns = "waypoint_route";
    line.type = Marker::LINE_STRIP;
    line.action = Marker::ADD;
    line.pose.orientation.w = 1;
    line.scale.x = 0.12;
    line.color.r = 0.2; line.color.g = 0.7; line.color.b = 1.0; line.color.a = 0.9;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      Marker arrow;
      arrow.header.frame_id = frame_;
      arrow.ns = "waypoint_heading";
      arrow.id = static_cast<int>(i);
      arrow.type = Marker::ARROW;
      arrow.action = Marker::ADD;
      arrow.pose = waypoints_[i].pose;
      arrow.pose.position.z = 0.15;
      arrow.scale.x = 1.0; arrow.scale.y = 0.2; arrow.scale.z = 0.2;
      arrow.color.a = 1;
      arrow.color.r = i < next_ ? 0.2 : 1.0;
      arrow.color.g = (i == next_ && active_ && waypoint_goal_) ? 0.4 : 0.9;
      arrow.color.b = 0.2;
      array.markers.push_back(arrow);
      Marker label = arrow;
      label.ns = "waypoint_number";
      label.type = Marker::TEXT_VIEW_FACING;
      label.pose.orientation = geometry_msgs::msg::Quaternion();
      label.pose.orientation.w = 1;
      label.pose.position.z = 1.0;
      label.scale.z = 0.8;
      label.text = std::to_string(i + 1);
      array.markers.push_back(label);
      line.points.push_back(arrow.pose.position);
    }
    if (line.points.size() > 1) array.markers.push_back(line);
    markers_pub_->publish(array);
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string frame_, base_frame_, state_;
  double max_distance_, permission_timeout_;
  int max_waypoints_;
  std::vector<Pose> waypoints_;
  size_t next_{0};
  bool allowed_{false}, request_pending_{false}, running_{false}, active_{false};
  bool waypoint_goal_{false}, cancel_requested_{false}, cancel_inflight_{false};
  int64_t request_id_{0};
  uint64_t sequence_{0}, goal_token_{0};
  Clock::time_point gps_time_{}, permission_time_{}, request_time_{}, sent_time_{}, cancel_time_{};
  std::optional<Pose> replacement_;
  Handle::SharedPtr handle_;
  rclcpp_action::Client<Nav>::SharedPtr nav_;
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr converter_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allowed_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<Pose>::SharedPtr rviz_sub_, waypoint_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, cancel_srv_, clear_srv_, undo_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalBridge>());
  rclcpp::shutdown();
}
