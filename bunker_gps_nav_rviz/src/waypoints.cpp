#include "bunker_gps_nav_rviz/waypoints.hpp"

#include <QGridLayout>
#include <QPointer>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace bunker_gps_nav_rviz {
AddWaypoint::AddWaypoint() {
  setName("Add Waypoint");
  shortcut_key_ = 'w';
  topic_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "/navigation/waypoint_input", "geometry_msgs/msg/PoseStamped",
    "Append a waypoint without starting navigation", getPropertyContainer(), SLOT(updateTopic()), this);
}
void AddWaypoint::onInitialize() {
  PoseTool::onInitialize();
  auto abstraction = context_->getRosNodeAbstraction().lock();
  node_ = abstraction->get_raw_node();
  topic_->initialize(abstraction);
  updateTopic();
}
void AddWaypoint::updateTopic() {
  if (node_) publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_->getTopicStd(), 10);
}
void AddWaypoint::onPoseSet(double x, double y, double theta) {
  if (!publisher_) return;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = context_->getFixedFrame().toStdString();
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = orientationAroundZAxis(theta);
  publisher_->publish(pose);
}

WaypointsPanel::WaypointsPanel(QWidget *parent) : Panel(parent) {
  auto layout = new QVBoxLayout(this);
  auto help = new QLabel("Add Waypoint (W): click and drag to set position and heading.", this);
  help->setWordWrap(true);
  layout->addWidget(help);
  status_ = new QLabel("Waiting for goal bridge...", this);
  status_->setWordWrap(true);
  layout->addWidget(status_);
  start_ = new QPushButton("Start / Resume", this);
  cancel_ = new QPushButton("Cancel", this);
  undo_ = new QPushButton("Undo Last", this);
  clear_ = new QPushButton("Clear", this);
  auto buttons = new QGridLayout;
  buttons->addWidget(start_, 0, 0);
  buttons->addWidget(cancel_, 0, 1);
  buttons->addWidget(undo_, 1, 0);
  buttons->addWidget(clear_, 1, 1);
  layout->addLayout(buttons);
  response_ = new QLabel(this);
  response_->setWordWrap(true);
  layout->addWidget(response_);
  layout->addStretch();
  connect(start_, &QPushButton::clicked, this, [this]() { request(start_client_, start_); });
  connect(cancel_, &QPushButton::clicked, this, [this]() { request(cancel_client_, cancel_); });
  connect(undo_, &QPushButton::clicked, this, [this]() { request(undo_client_, undo_); });
  connect(clear_, &QPushButton::clicked, this, [this]() { request(clear_client_, clear_); });
}
void WaypointsPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  connectServices();
}
void WaypointsPanel::connectServices() {
  if (!node_) return;
  const auto prefix = service_namespace_.toStdString();
  start_client_ = node_->create_client<Trigger>(prefix + "/start");
  cancel_client_ = node_->create_client<Trigger>(prefix + "/cancel");
  undo_client_ = node_->create_client<Trigger>(prefix + "/remove_last");
  clear_client_ = node_->create_client<Trigger>(prefix + "/clear");
  QPointer<QLabel> label(status_);
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(prefix + "/status",
    rclcpp::QoS(1).transient_local(), [label](std_msgs::msg::String::ConstSharedPtr m) {
      if (!label) return;
      const QString text = QString::fromStdString(m->data);
      QMetaObject::invokeMethod(label, [label, text]() { if (label) label->setText(text); }, Qt::QueuedConnection);
    });
}
void WaypointsPanel::request(const rclcpp::Client<Trigger>::SharedPtr &client, QPushButton *button) {
  if (!client || !client->service_is_ready()) {
    response_->setText("Goal bridge service is unavailable.");
    return;
  }
  button->setEnabled(false);
  QPointer<QLabel> label(response_);
  QPointer<QPushButton> control(button);
  auto pending = std::make_shared<bool>(true);  // Only read/written on the Qt thread.
  auto future = client->async_send_request(std::make_shared<Trigger::Request>(),
    [label, control, pending](rclcpp::Client<Trigger>::SharedFuture result) {
      const auto response = result.get();
      const QString text = QString::fromStdString(response->message);
      if (!label) return;
      QMetaObject::invokeMethod(label, [label, control, pending, text]() {
        if (!*pending) return;
        *pending = false;
        if (label) label->setText(text);
        if (control) control->setEnabled(true);
      }, Qt::QueuedConnection);
    });
  QTimer::singleShot(5000, this, [client, id = future.request_id, label, control, pending]() {
    if (!*pending) return;
    *pending = false;
    client->remove_pending_request(id);
    if (label) label->setText("Service response timed out; check navigation status.");
    if (control) control->setEnabled(true);
  });
}
void WaypointsPanel::save(rviz_common::Config config) const {
  Panel::save(config);
  config.mapSetValue("Service Namespace", service_namespace_);
}
void WaypointsPanel::load(const rviz_common::Config &config) {
  Panel::load(config);
  config.mapGetString("Service Namespace", &service_namespace_);
  connectServices();
}
}  // namespace bunker_gps_nav_rviz

PLUGINLIB_EXPORT_CLASS(bunker_gps_nav_rviz::AddWaypoint, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(bunker_gps_nav_rviz::WaypointsPanel, rviz_common::Panel)
