#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

bool parseDouble(const std::string & text, double & value)
{
  char * end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return end != text.c_str() && *end == '\0';
}

void printUsage()
{
  std::cerr
    << "Usage:\n"
    << "  ros2 run nav2_waypoint_runner_cpp publish_waypoints [--topic /nav2_waypoints]"
    << " [--frame map] x y yaw [x y yaw ...]\n"
    << "\n"
    << "Example:\n"
    << "  ros2 run nav2_waypoint_runner_cpp publish_waypoints -- "
    << "--frame map -1.03 3.43 1.57 -4.85 5.13 0.0\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string topic = "/nav2_waypoints";
  std::string frame = "map";
  std::vector<double> numbers;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--") {
      continue;
    }
    if (arg == "--topic" && i + 1 < argc) {
      topic = argv[++i];
      continue;
    }
    if (arg == "--frame" && i + 1 < argc) {
      frame = argv[++i];
      continue;
    }
    if (arg == "--help" || arg == "-h") {
      printUsage();
      return 0;
    }

    double value = 0.0;
    if (!parseDouble(arg, value)) {
      std::cerr << "Invalid argument: " << arg << "\n";
      printUsage();
      return 2;
    }
    numbers.push_back(value);
  }

  if (numbers.empty() || numbers.size() % 3 != 0) {
    std::cerr << "Expected x y yaw triples.\n";
    printUsage();
    return 2;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("nav2_waypoint_publisher_cpp");
  auto pub = node->create_publisher<geometry_msgs::msg::PoseArray>(
    topic, rclcpp::QoS(1).transient_local().reliable());

  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = frame;
  msg.header.stamp = node->now();
  msg.poses.reserve(numbers.size() / 3);

  for (size_t i = 0; i < numbers.size(); i += 3) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = numbers[i];
    pose.position.y = numbers[i + 1];
    pose.orientation = yawToQuaternion(numbers[i + 2]);
    msg.poses.push_back(pose);
  }

  RCLCPP_INFO(
    node->get_logger(), "Publishing %zu waypoint(s) to %s in frame %s",
    msg.poses.size(), topic.c_str(), frame.c_str());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && pub->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  pub->publish(msg);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  rclcpp::shutdown();
  return 0;
}
