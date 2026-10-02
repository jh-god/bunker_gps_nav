#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto args=rclcpp::remove_ros_arguments(argc,argv);std::map<std::string,double> values;
    for (size_t i=1;i<args.size();i+=2) {
      if (i+1>=args.size() || (args[i]!="--lat" && args[i]!="--lon" && args[i]!="--heading" && args[i]!="--alt"))
        throw std::invalid_argument("Usage: send_gps_goal --lat LAT --lon LON --heading DEGREES [--alt METRES]");
      size_t consumed=0;double v=std::stod(args[i+1],&consumed);
      if (consumed!=args[i+1].size() || !std::isfinite(v)) throw std::invalid_argument("Invalid number");
      values[args[i]]=v;
    }
    if (!values.count("--lat") || !values.count("--lon") || !values.count("--heading") ||
        std::abs(values["--lat"])>90 || std::abs(values["--lon"])>180)
      throw std::invalid_argument("Provide valid --lat --lon --heading (compass degrees, North=0, East=90)");
    auto node=std::make_shared<rclcpp::Node>("send_gps_goal");
    auto pub=node->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/gps_goal",10);
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (rclcpp::ok() && pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline) {
      rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (pub->get_subscription_count()==0) throw std::runtime_error("No GPS goal bridge subscriber");
    geographic_msgs::msg::GeoPoseStamped m;m.header.stamp=node->now();m.header.frame_id="earth";
    m.pose.position.latitude=values["--lat"];m.pose.position.longitude=values["--lon"];m.pose.position.altitude=values["--alt"];
    constexpr double pi=3.14159265358979323846;double yaw=pi/2-values["--heading"]*pi/180;
    m.pose.orientation.z=std::sin(yaw/2);m.pose.orientation.w=std::cos(yaw/2);pub->publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout<<"GPS goal published; check gps_goal_bridge_node for acceptance.\n";
  } catch (const std::exception& e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
