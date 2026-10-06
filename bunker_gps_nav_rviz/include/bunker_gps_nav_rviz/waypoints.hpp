#pragma once

#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace bunker_gps_nav_rviz {
class AddWaypoint : public rviz_default_plugins::tools::PoseTool {
  Q_OBJECT
 public:
  AddWaypoint();
  void onInitialize() override;
 protected:
  void onPoseSet(double x, double y, double theta) override;
 private Q_SLOTS:
  void updateTopic();
 private:
  rviz_common::properties::RosTopicProperty *topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

class WaypointsPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit WaypointsPanel(QWidget *parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;
 private:
  using Trigger = std_srvs::srv::Trigger;
  void connectServices();
  void request(const rclcpp::Client<Trigger>::SharedPtr &client, QPushButton *button);
  QString service_namespace_{"/navigation/waypoints"};
  QLabel *status_, *response_;
  QPushButton *start_, *cancel_, *undo_, *clear_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr start_client_, cancel_client_, undo_client_, clear_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};
}  // namespace bunker_gps_nav_rviz
