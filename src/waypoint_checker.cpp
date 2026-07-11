#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nlohmann/json.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

namespace nav2_waypoint_runner_cpp
{

class WaypointChecker : public rclcpp::Node
{
public:
  WaypointChecker()
  : Node("nav2_waypoint_checker_cpp")
  {
//参数声明
    json_points_topic_ = declare_parameter<std::string>(
      "json_points_topic",
      "/waypoint_points_json");

    output_waypoints_topic_ = declare_parameter<std::string>(
    "output_waypoints_topic",
    "/nav2_waypoints");

//创建地图和json点订阅者
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(
        &WaypointChecker::mapCallback,
        this,
        std::placeholders::_1));

    json_points_sub_ = create_subscription<std_msgs::msg::String>(
      json_points_topic_,
      10,
      std::bind(
        &WaypointChecker::jsonPointsCallback,
        this,
        std::placeholders::_1));

    waypoints_pub_ =
    create_publisher<geometry_msgs::msg::PoseArray>(
    output_waypoints_topic_,
    10);

    RCLCPP_INFO(
      get_logger(),
      "Waypoint coordinate converter started.");

    RCLCPP_INFO(
      get_logger(),
      "JSON input topic: %s",
      json_points_topic_.c_str());
  }

private:

//像素坐标
  struct PixelPoint
  {
    double u{0.0};
    double v{0.0};

    std::optional<double> yaw;
    std::string name;
  };

//map坐标
  struct MapPoint
  {
    double x{0.0};
    double y{0.0};

    std::optional<double> yaw;

    std::string name;
  };

  //栅格坐标
  struct GridPoint
  {
    int x{0};
    int y{0};
  };

  void mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_width_ = static_cast<int>(msg->info.width);
    map_height_ = static_cast<int>(msg->info.height);
    map_resolution_ = static_cast<double>(msg->info.resolution);

    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;

    map_origin_yaw_ = quaternionToYaw(
      msg->info.origin.orientation.x,
      msg->info.origin.orientation.y,
      msg->info.origin.orientation.z,
      msg->info.origin.orientation.w);

    map_frame_ = msg->header.frame_id.empty() ?
      "map" :
      msg->header.frame_id;

    if (map_width_ <= 0 ||
      map_height_ <= 0 ||
      map_resolution_ <= 0.0)
    {
      map_received_ = false;

      RCLCPP_ERROR(
        get_logger(),
        "Received invalid map metadata: width=%d, height=%d, "
        "resolution=%.6f",
        map_width_,
        map_height_,
        map_resolution_);

      return;
    }

    map_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Received map: width=%d, height=%d, resolution=%.3f, "
      "origin=(%.3f, %.3f, %.6f), frame=%s",
      map_width_,
      map_height_,
      map_resolution_,
      map_origin_x_,
      map_origin_y_,
      map_origin_yaw_,
      map_frame_.c_str());

    // 如果地图到来前已经收到了前端任务，
    // 现在依次处理缓存的任务。
    while (!pending_batches_.empty()) {
      auto points = std::move(pending_batches_.front());
      pending_batches_.pop_front();

      processPoints(points);
    }
  }

  void jsonPointsCallback(
    const std_msgs::msg::String::SharedPtr msg)
  {
    std::vector<PixelPoint> points;

    try {
      const auto json_data =
        nlohmann::json::parse(msg->data);

      points = parsePoints(json_data);
    } catch (const nlohmann::json::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid points JSON: %s",
        error.what());

      return;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to parse points: %s",
        error.what());

      return;
    }

    if (points.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Received JSON, but no valid pixel points were found");

      return;
    }

    if (!map_received_) {
      pending_batches_.push_back(points);

      RCLCPP_WARN(
        get_logger(),
        "Received %zu point(s), but /map is not ready. "
        "This waypoint batch has been cached.",
        points.size());

      return;
    }

    processPoints(points);
  }


  std::vector<PixelPoint> parsePoints(
    const nlohmann::json & json_data) const
  {
    std::vector<PixelPoint> points;

    if (json_data.is_object() &&
      json_data.contains("points"))
    {
      return parsePoints(json_data.at("points"));
    }

    if (json_data.is_array()) {
      for (const auto & item : json_data) {
        const auto point = parsePoint(item);

        if (point.has_value()) {
          points.push_back(point.value());
        }
      }

      return points;
    }

    const auto point = parsePoint(json_data);

    if (point.has_value()) {
      points.push_back(point.value());
    }

    return points;
  }


  std::optional<PixelPoint> parsePoint(
    const nlohmann::json & json_data) const
  {
    if (!json_data.is_object()) {
      return std::nullopt;
    }

    PixelPoint point;

    if (json_data.contains("pixel_x") &&
      json_data.contains("pixel_y"))
    {
      point.u = json_data.at("pixel_x").get<double>();
      point.v = json_data.at("pixel_y").get<double>();
    } else if (json_data.contains("u") &&
      json_data.contains("v"))
    {
      point.u = json_data.at("u").get<double>();
      point.v = json_data.at("v").get<double>();
    } else {
      return std::nullopt;
    }

    if (!std::isfinite(point.u) ||
      !std::isfinite(point.v))
    {
      return std::nullopt;
    }

    point.name = json_data.value("name", "");

    if (json_data.contains("yaw") &&
      !json_data.at("yaw").is_null())
    {
      const double yaw =
        json_data.at("yaw").get<double>();

      if (!std::isfinite(yaw)) {
        return std::nullopt;
      }

      point.yaw = yaw;
    }

    return point;
  }


void processPoints(
  const std::vector<PixelPoint> & pixel_points)
{
  if (!map_received_) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot process waypoints because map is unavailable");
    return;
  }

  if (map_resolution_ <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot process waypoints because map resolution is invalid");
    return;
  }

  // 最终要发布的PoseArray
  geometry_msgs::msg::PoseArray pose_array;

  pose_array.header.frame_id = map_frame_;
  pose_array.header.stamp = this->now();

  pose_array.poses.reserve(pixel_points.size());

  for (size_t index = 0;
    index < pixel_points.size();
    ++index)
  {
    const auto & pixel_point = pixel_points.at(index);

    const std::string point_name =
      pixel_point.name.empty() ?
      "point_" + std::to_string(index) :
      pixel_point.name;

    // 像素坐标转换为map米制坐标
    const auto map_point_result =
      pixelToMap(pixel_point);

    if (!map_point_result.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "%s pixel coordinate is outside map image: "
        "pixel=(%.3f, %.3f), image_size=%d x %d",
        point_name.c_str(),
        pixel_point.u,
        pixel_point.v,
        map_width_,
        map_height_);

      continue;
    }

    const MapPoint map_point =
      map_point_result.value();

    geometry_msgs::msg::Pose pose;

    pose.position.x = map_point.x;
    pose.position.y = map_point.y;
    pose.position.z = 0.0;

    /*
     * 前端提供yaw时，转换为四元数。
     *
     * 前端没有提供yaw时，将四元数保持为全0。
     * 如果WaypointSubscriber的fill_missing_yaw=true，
     * 它会在onPoseArray()之后自动计算朝向。
     */
    if (map_point.yaw.has_value()) {
      const double yaw = map_point.yaw.value();

      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = std::sin(yaw * 0.5);
      pose.orientation.w = std::cos(yaw * 0.5);
    } else {
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 0.0;
    }

    pose_array.poses.push_back(pose);

    RCLCPP_INFO(
      get_logger(),
      "%s converted: pixel=(%.3f, %.3f), "
      "map=(%.3f, %.3f)",
      point_name.c_str(),
      pixel_point.u,
      pixel_point.v,
      map_point.x,
      map_point.y);
  }

  if (pose_array.poses.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "No valid waypoint remains. PoseArray will not be published.");
    return;
  }

  // 一次发布完整的多点数组
  waypoints_pub_->publish(pose_array);

  RCLCPP_INFO(
    get_logger(),
    "Published %zu waypoint(s) to %s, frame=%s",
    pose_array.poses.size(),
    output_waypoints_topic_.c_str(),
    pose_array.header.frame_id.c_str());
}

  //原始地图图片像素 → map米制坐标
  std::optional<MapPoint> pixelToMap(
    const PixelPoint & pixel_point) const
  {
    // u、v必须是原始地图图片的像素坐标
    if (pixel_point.u < 0.0 ||
      pixel_point.v < 0.0 ||
      pixel_point.u >= static_cast<double>(map_width_) ||
      pixel_point.v >= static_cast<double>(map_height_))
    {
      return std::nullopt;
    }

    const double local_x =
      (pixel_point.u + 0.5) *
      map_resolution_;

    const double local_y =
      (
      static_cast<double>(map_height_) -
      pixel_point.v -
      0.5
      ) *
      map_resolution_;

    const double cos_yaw =
      std::cos(map_origin_yaw_);

    const double sin_yaw =
      std::sin(map_origin_yaw_);

    MapPoint map_point;

    map_point.x =
      map_origin_x_ +
      cos_yaw * local_x -
      sin_yaw * local_y;

    map_point.y =
      map_origin_y_ +
      sin_yaw * local_x +
      cos_yaw * local_y;

    map_point.yaw = pixel_point.yaw;
    map_point.name = pixel_point.name;

    return map_point;
  }


  bool mapToGrid(
    double map_x,
    double map_y,
    GridPoint & grid_point) const
  {
    if (map_resolution_ <= 0.0) {
      return false;
    }

    const double dx =
      map_x - map_origin_x_;

    const double dy =
      map_y - map_origin_y_;

    const double cos_yaw =
      std::cos(map_origin_yaw_);

    const double sin_yaw =
      std::sin(map_origin_yaw_);

    const double local_x =
      cos_yaw * dx +
      sin_yaw * dy;

    const double local_y =
      -sin_yaw * dx +
      cos_yaw * dy;

    grid_point.x = static_cast<int>(
      std::floor(local_x / map_resolution_));

    grid_point.y = static_cast<int>(
      std::floor(local_y / map_resolution_));

    return isGridInsideMap(
      grid_point.x,
      grid_point.y);
  }



  bool isGridInsideMap(
    int grid_x,
    int grid_y) const
  {
    return
      grid_x >= 0 &&
      grid_y >= 0 &&
      grid_x < map_width_ &&
      grid_y < map_height_;
  }

  // 四元数 → yaw

  static double quaternionToYaw(
    double x,
    double y,
    double z,
    double w)
  {

    const double sin_yaw =
      2.0 * (w * z + x * y);

    const double cos_yaw =
      1.0 -
      2.0 * (y * y + z * z);

    return std::atan2(
      sin_yaw,
      cos_yaw);
  }


  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr json_points_sub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;


  int map_width_{0};
  int map_height_{0};

  double map_resolution_{0.0};

  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_origin_yaw_{0.0};

  std::string map_frame_{"map"};

  bool map_received_{false};


  std::string json_points_topic_;
  std::string output_waypoints_topic_;

  // 地图未收到时缓存的前端任务。
  std::deque<std::vector<PixelPoint>> pending_batches_;
};

}  // namespace nav2_waypoint_runner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        nav2_waypoint_runner_cpp::WaypointChecker>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "nav2_waypoint_checker_cpp"),
      "Node terminated with exception: %s",
      error.what());
  }

  rclcpp::shutdown();
  return 0;
}
