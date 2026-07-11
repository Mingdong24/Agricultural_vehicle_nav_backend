#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_waypoint_runner_cpp
{

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

bool quaternionIsEmpty(const geometry_msgs::msg::Quaternion & q)
{
  return q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0;
}

double yawBetween(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  return std::atan2(b.position.y - a.position.y, b.position.x - a.position.x);
}

class WaypointSubscriber : public rclcpp::Node
{
public:
  WaypointSubscriber()
  : Node("nav2_waypoint_subscriber_cpp")
  {
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/nav2_waypoints");
    path_topic_ = declare_parameter<std::string>("path_topic", "/nav2_waypoint_path");
    append_topic_ = declare_parameter<std::string>("append_topic", "/nav2_waypoint_append");
    action_name_ = declare_parameter<std::string>("action_name", "/follow_waypoints");
    default_frame_ = declare_parameter<std::string>("default_frame", "map");
    auto_start_ = declare_parameter<bool>("auto_start", true);
    cancel_on_new_goal_ = declare_parameter<bool>("cancel_on_new_goal", true);
    fill_missing_yaw_ = declare_parameter<bool>("fill_missing_yaw", true);

    action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, action_name_);

    pose_array_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      waypoint_topic_, 10,
      std::bind(&WaypointSubscriber::onPoseArray, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&WaypointSubscriber::onPath, this, std::placeholders::_1));

    append_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      append_topic_, 10,
      std::bind(&WaypointSubscriber::onAppendPose, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Listening: PoseArray %s, Path %s, append PoseStamped %s; sending to %s",
      waypoint_topic_.c_str(), path_topic_.c_str(), append_topic_.c_str(), action_name_.c_str());
  }

private:
  void onPoseArray(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    const std::string frame_id = msg->header.frame_id.empty() ? default_frame_ : msg->header.frame_id;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      poses.push_back(makePoseStamped(pose, frame_id));
    }
    prepareOrSend(std::move(poses), "PoseArray");
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const std::string path_frame = msg->header.frame_id.empty() ? default_frame_ : msg->header.frame_id;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(msg->poses.size());
    for (const auto & pose_stamped : msg->poses) {
      const std::string frame_id =
        pose_stamped.header.frame_id.empty() ? path_frame : pose_stamped.header.frame_id;
      poses.push_back(makePoseStamped(pose_stamped.pose, frame_id));
    }
    prepareOrSend(std::move(poses), "Path");
  }

  void onAppendPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const std::string frame_id = msg->header.frame_id.empty() ? default_frame_ : msg->header.frame_id;
    buffered_waypoints_.push_back(makePoseStamped(msg->pose, frame_id));
    RCLCPP_INFO(
      get_logger(),
      "Buffered waypoint %zu: x=%.3f, y=%.3f, frame=%s",
      buffered_waypoints_.size(), msg->pose.position.x, msg->pose.position.y, frame_id.c_str());

    if (auto_start_ && goal_handle_ == nullptr) {
      auto poses = buffered_waypoints_;
      buffered_waypoints_.clear();
      prepareOrSend(std::move(poses), "PoseStamped append buffer");
    }
  }

  geometry_msgs::msg::PoseStamped makePoseStamped(
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id)
  {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = frame_id.empty() ? default_frame_ : frame_id;
    stamped.header.stamp = now();
    stamped.pose = pose;
    return stamped;
  }

  void prepareOrSend(
    std::vector<geometry_msgs::msg::PoseStamped> poses,
    const std::string & source)
  {
    if (poses.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring empty waypoint list from %s", source.c_str());
      return;
    }

    if (fill_missing_yaw_) {
      fillYawFromPath(poses);
    }

    logWaypoints(poses, source);

    if (!auto_start_) {
      pending_goal_ = poses;
      RCLCPP_INFO(get_logger(), "auto_start is false; waypoints stored as pending goal");
      return;
    }

    sendOrReplaceGoal(std::move(poses));
  }

  void fillYawFromPath(std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    double last_yaw = 0.0;
    for (size_t i = 0; i < poses.size(); ++i) {
      if (!quaternionIsEmpty(poses[i].pose.orientation)) {
        continue;
      }
      if (i + 1 < poses.size()) {
        last_yaw = yawBetween(poses[i].pose, poses[i + 1].pose);
      }
      poses[i].pose.orientation = yawToQuaternion(last_yaw);
    }
  }

  void logWaypoints(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    const std::string & source)
  {
    const auto & first = poses.front().pose.position;
    const auto & last = poses.back().pose.position;
    const auto & frame = poses.front().header.frame_id;
    RCLCPP_INFO(
      get_logger(),
      "Received %zu waypoint(s) from %s, frame=%s, first=(%.3f, %.3f), last=(%.3f, %.3f)",
      poses.size(), source.c_str(), frame.c_str(), first.x, first.y, last.x, last.y);

    for (const auto & pose : poses) {
      if (pose.header.frame_id != frame) {
        RCLCPP_WARN(get_logger(), "Waypoint list contains mixed frame_id values");
        break;
      }
    }

    if (frame != "map") {
      RCLCPP_WARN(
        get_logger(),
        "Waypoint frame is '%s', but this robot's Nav2 global frame is configured as 'map'",
        frame.c_str());
    }
  }

  void sendOrReplaceGoal(std::vector<geometry_msgs::msg::PoseStamped> poses)
  {
    if (goal_handle_ != nullptr) {
      if (cancel_on_new_goal_) {
        RCLCPP_WARN(get_logger(), "Active waypoint task exists; canceling it and sending new goal");
        (void)action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
        sendGoal(std::move(poses));
      } else {
        RCLCPP_WARN(get_logger(), "Active waypoint task exists; ignoring new goal");
      }
      return;
    }

    sendGoal(std::move(poses));
  }

  void sendGoal(std::vector<geometry_msgs::msg::PoseStamped> poses)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Nav2 /follow_waypoints action server is not available");
      return;
    }

    FollowWaypoints::Goal goal;
    goal.poses = std::move(poses);

    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&WaypointSubscriber::onGoalResponse, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(
      &WaypointSubscriber::onFeedback, this, std::placeholders::_1,
      std::placeholders::_2);
    options.result_callback =
      std::bind(&WaypointSubscriber::onResult, this, std::placeholders::_1);

    const size_t count = goal.poses.size();
    action_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "Sent %zu waypoint(s) to Nav2", count);
  }

  void onGoalResponse(const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      goal_handle_.reset();
      RCLCPP_ERROR(get_logger(), "Waypoint goal was rejected by Nav2");
      return;
    }
    goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "Waypoint goal accepted");
  }

  void onFeedback(
    GoalHandleFollowWaypoints::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Current waypoint index: %u", feedback->current_waypoint);
  }

  void onResult(const GoalHandleFollowWaypoints::WrappedResult & result)
  {
    goal_handle_.reset();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->missed_waypoints.empty())
    {
      RCLCPP_INFO(get_logger(), "Waypoint task completed successfully");
    } else {
      std::string missed;
      for (auto index : result.result->missed_waypoints) {
        missed += std::to_string(index) + " ";
      }
      RCLCPP_WARN(
        get_logger(), "Waypoint task ended with result_code=%d, missed_waypoints=[%s]",
        static_cast<int>(result.code), missed.c_str());
    }

    if (pending_goal_.has_value()) {
      auto pending = pending_goal_.value();
      pending_goal_.reset();
      sendGoal(std::move(pending));
    }
  }

  std::string waypoint_topic_;
  std::string path_topic_;
  std::string append_topic_;
  std::string action_name_;
  std::string default_frame_;
  bool auto_start_;
  bool cancel_on_new_goal_;
  bool fill_missing_yaw_;

  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  GoalHandleFollowWaypoints::SharedPtr goal_handle_;
  std::optional<std::vector<geometry_msgs::msg::PoseStamped>> pending_goal_;
  std::vector<geometry_msgs::msg::PoseStamped> buffered_waypoints_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr append_sub_;
};

}  // namespace nav2_waypoint_runner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_waypoint_runner_cpp::WaypointSubscriber>());
  rclcpp::shutdown();
  return 0;
}
