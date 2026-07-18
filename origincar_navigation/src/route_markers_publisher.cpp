#include <cmath>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kDegreesToRadians = M_PI / 180.0;

// RViz Marker 只需要路线关键点的位置和朝向；这里不用 PoseStamped，
// 因为 frame_id 由 Marker 统一指定。
struct RoutePoint
{
  double x{0.0};
  double y{0.0};
  geometry_msgs::msg::Quaternion orientation;
};

struct Route
{
  std::string name;
  std::vector<RoutePoint> points;
};

double readRequiredDouble(
  const YAML::Node & node,
  const std::string & field_name,
  const std::string & route_name,
  size_t point_index)
{
  // 可视化节点和全局规划器读取同一份 global_routes.yaml。
  // 缺少 x/y/orientation 时直接报错，方便发现路线文件配置问题。
  if (!node[field_name]) {
    throw std::runtime_error(
            "Missing '" + field_name + "' in " + route_name +
            " point " + std::to_string(point_index));
  }

  return node[field_name].as<double>();
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  // YAML 支持 yaw/yaw_deg 两种简写形式，最终都转换成 Marker 使用的 quaternion。
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

std_msgs::msg::ColorRGBA colorForRoute(size_t index, float alpha = 1.0F)
{
  // 给不同路线分配固定循环颜色，便于在 RViz 中区分 route_1/2/3。
  std_msgs::msg::ColorRGBA color;
  color.a = alpha;

  switch (index % 3) {
    case 0:
      color.r = 0.10F;
      color.g = 0.58F;
      color.b = 1.00F;
      break;
    case 1:
      color.r = 0.10F;
      color.g = 0.82F;
      color.b = 0.38F;
      break;
    default:
      color.r = 1.00F;
      color.g = 0.48F;
      color.b = 0.12F;
      break;
  }

  return color;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

RoutePoint parsePoint(
  const YAML::Node & point_node,
  const std::string & route_name,
  size_t point_index)
{
  // 支持三种朝向写法：
  // 1. orientation: {x, y, z, w}，适合从 RViz echo 结果直接粘贴；
  // 2. yaw_deg，适合人工编辑；
  // 3. yaw，适合程序生成。
  RoutePoint point;
  point.x = readRequiredDouble(point_node, "x", route_name, point_index);
  point.y = readRequiredDouble(point_node, "y", route_name, point_index);

  if (point_node["orientation"]) {
    const auto orientation = point_node["orientation"];
    point.orientation.x = readRequiredDouble(orientation, "x", route_name, point_index);
    point.orientation.y = readRequiredDouble(orientation, "y", route_name, point_index);
    point.orientation.z = readRequiredDouble(orientation, "z", route_name, point_index);
    point.orientation.w = readRequiredDouble(orientation, "w", route_name, point_index);
  } else if (point_node["yaw_deg"]) {
    point.orientation = quaternionFromYaw(point_node["yaw_deg"].as<double>() * kDegreesToRadians);
  } else if (point_node["yaw"]) {
    point.orientation = quaternionFromYaw(point_node["yaw"].as<double>());
  } else {
    throw std::runtime_error(
            "Point " + std::to_string(point_index) + " in " + route_name +
            " must define orientation quaternion, yaw_deg, or yaw");
  }

  return point;
}

std::vector<Route> loadRoutes(const std::string & file_path)
{
  // 每次发布前重新读取 YAML。这样调试路线时修改文件后无需重启节点，
  // 等下一个 timer 周期就能在 RViz 里看到变化。
  const auto config = YAML::LoadFile(file_path);
  const auto routes_node = config["routes"];
  if (!routes_node || !routes_node.IsMap()) {
    throw std::runtime_error("Route config must contain a 'routes' map");
  }

  std::vector<Route> routes;
  for (const auto & route_entry : routes_node) {
    Route route;
    route.name = route_entry.first.as<std::string>();

    const auto points_node = route_entry.second["points"];
    if (!points_node || !points_node.IsSequence()) {
      throw std::runtime_error(route.name + " must contain a points sequence");
    }

    route.points.reserve(points_node.size());
    for (size_t i = 0; i < points_node.size(); ++i) {
      route.points.push_back(parsePoint(points_node[i], route.name, i));
    }

    routes.push_back(route);
  }

  return routes;
}

}  // namespace

class RouteMarkersPublisher : public rclcpp::Node
{
public:
  RouteMarkersPublisher()
  : Node("route_markers_publisher")
  {
    // 默认读取安装后的 global_routes.yaml；也可通过 routes_file 参数临时指定其他文件。
    const auto default_routes_file =
      ament_index_cpp::get_package_share_directory("origincar_navigation") +
      "/config/global_routes.yaml";

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    routes_file_ = declare_parameter<std::string>("routes_file", default_routes_file);
    marker_topic_ = declare_parameter<std::string>("marker_topic", "global_route_markers");
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 1000);

    // transient_local 让 RViz 后启动也能立即收到最近一次 MarkerArray。
    publisher_ = create_publisher<MarkerArray>(marker_topic_, rclcpp::QoS(1).transient_local());
    publishMarkers();

    // 定时重发有两个作用：
    // 1. RViz/订阅者中途重连后能看到路线；
    // 2. 修改 YAML 后可自动刷新显示。
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(100, publish_period_ms_)),
      [this]() {publishMarkers();});
  }

private:
  void publishMarkers()
  {
    MarkerArray marker_array;

    try {
      const auto routes = loadRoutes(routes_file_);
      // 先发 DELETEALL 清除旧 Marker，避免路线点数量变少后旧 Marker 残留。
      addDeleteAllMarker(marker_array);

      int marker_id = 1;
      for (size_t route_index = 0; route_index < routes.size(); ++route_index) {
        addRouteMarkers(routes[route_index], route_index, marker_id, marker_array);
      }

      publisher_->publish(marker_array);
      RCLCPP_DEBUG(
        get_logger(), "Published %zu route marker messages from %s",
        marker_array.markers.size(), routes_file_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to publish route markers from '%s': %s",
        routes_file_.c_str(), ex.what());
    }
  }

  void addDeleteAllMarker(MarkerArray & marker_array) const
  {
    // DELETEALL 只需要 ns/id/action，RViz 收到后会清理该 topic 下旧的 Marker。
    Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "global_routes";
    marker.id = 0;
    marker.action = Marker::DELETEALL;
    marker_array.markers.push_back(marker);
  }

  void addRouteMarkers(
    const Route & route,
    size_t route_index,
    int & marker_id,
    MarkerArray & marker_array) const
  {
    const auto color = colorForRoute(route_index);

    // LINE_STRIP 连接路线关键点，只表示 YAML 中配置的控制点顺序，
    // 不是全局规划器最终生成的平滑插值路径。
    Marker line_marker;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = now();
    line_marker.ns = route.name + "_line";
    line_marker.id = marker_id++;
    line_marker.type = Marker::LINE_STRIP;
    line_marker.action = Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.035;
    line_marker.color = colorForRoute(route_index, 0.9F);

    for (const auto & point : route.points) {
      line_marker.points.push_back(makePoint(point.x, point.y, 0.04));
    }
    marker_array.markers.push_back(line_marker);

    for (size_t i = 0; i < route.points.size(); ++i) {
      const auto & point = route.points[i];
      // 每个关键点显示三类 Marker：球体表示位置，箭头表示朝向，文字表示编号。
      addSphere(route, point, i, color, marker_id, marker_array);
      addArrow(route, point, i, color, marker_id, marker_array);
      addLabel(route, point, i, color, marker_id, marker_array);
    }
  }

  void addSphere(
    const Route & route,
    const RoutePoint & point,
    size_t point_index,
    const std_msgs::msg::ColorRGBA & color,
    int & marker_id,
    MarkerArray & marker_array) const
  {
    // 球体用于标记路线关键点位置；第一个点稍大，便于识别路线起点。
    Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = route.name + "_points";
    marker.id = marker_id++;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;
    marker.pose.position = makePoint(point.x, point.y, 0.08);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.14;
    marker.scale.y = 0.14;
    marker.scale.z = 0.14;
    marker.color = color;
    if (point_index == 0) {
      marker.scale.x = 0.18;
      marker.scale.y = 0.18;
      marker.scale.z = 0.18;
    }
    marker_array.markers.push_back(marker);
  }

  void addArrow(
    const Route & route,
    const RoutePoint & point,
    size_t,
    const std_msgs::msg::ColorRGBA & color,
    int & marker_id,
    MarkerArray & marker_array) const
  {
    // 箭头直接使用 YAML 配置的 orientation，检查路线朝向是否录入正确。
    Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = route.name + "_directions";
    marker.id = marker_id++;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose.position = makePoint(point.x, point.y, 0.12);
    marker.pose.orientation = point.orientation;
    marker.scale.x = 0.32;
    marker.scale.y = 0.055;
    marker.scale.z = 0.055;
    marker.color = color;
    marker_array.markers.push_back(marker);
  }

  void addLabel(
    const Route & route,
    const RoutePoint & point,
    size_t point_index,
    const std_msgs::msg::ColorRGBA & color,
    int & marker_id,
    MarkerArray & marker_array) const
  {
    // TEXT_VIEW_FACING 会始终面向 RViz 相机，用 route_name_index 标出关键点顺序。
    Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = route.name + "_labels";
    marker.id = marker_id++;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.action = Marker::ADD;
    marker.pose.position = makePoint(point.x, point.y, 0.34);
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.16;
    marker.color = color;
    marker.text = route.name + "_" + std::to_string(point_index + 1);
    marker_array.markers.push_back(marker);
  }

  std::string frame_id_;
  std::string routes_file_;
  std::string marker_topic_;
  int publish_period_ms_{1000};
  rclcpp::Publisher<MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RouteMarkersPublisher>());
  rclcpp::shutdown();
  return 0;
}
