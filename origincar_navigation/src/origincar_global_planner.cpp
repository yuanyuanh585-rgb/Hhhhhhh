#include "origincar_navigation/origincar_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace origincar_navigation
{
namespace
{

// RViz/目标点的 z 值被用作路线编号或特殊规划模式时允许一点浮点误差。
// z 不是实际高度，只是本项目为了选择 route_N 或动态过渡规划做的编码。
constexpr double kGoalRouteTolerance = 0.01;
constexpr double kDynamicReverseGoalCode = -1.0;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr unsigned char kTransitionBlockedCost = 253;

// 简单二维向量工具，供路径平滑时计算方向、切线和插值导数。
struct Vector2
{
  double x{0.0};
  double y{0.0};
};

Vector2 operator+(const Vector2 & lhs, const Vector2 & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

Vector2 operator-(const Vector2 & lhs, const Vector2 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

Vector2 operator*(const Vector2 & lhs, double scale)
{
  return {lhs.x * scale, lhs.y * scale};
}

double dot(const Vector2 & lhs, const Vector2 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

double norm(const Vector2 & value)
{
  return std::hypot(value.x, value.y);
}

double distance(const Vector2 & lhs, const Vector2 & rhs)
{
  return norm(lhs - rhs);
}

Vector2 normalized(const Vector2 & value, const Vector2 & fallback = {1.0, 0.0})
{
  // 遇到重合点时无法归一化，返回调用方给定的备用方向，避免产生 NaN。
  const double length = norm(value);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return fallback;
  }

  return {value.x / length, value.y / length};
}

double clampedAcos(double value)
{
  // 浮点误差可能让点积略微超过 [-1, 1]，先夹紧再 acos。
  return std::acos(std::clamp(value, -1.0, 1.0));
}

// 从 yaw 生成四元数。路线配置支持 yaw/yaw_deg，也支持直接给 quaternion。
geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

double readRequiredDouble(
  const YAML::Node & node,
  const std::string & field_name,
  const std::string & route_name,
  size_t point_index)
{
  // YAML 路线点缺字段时立即抛错，避免生成一条“看似存在但坐标错误”的路径。
  if (!node[field_name]) {
    throw std::runtime_error(
            "Missing '" + field_name + "' in " + route_name +
            " point " + std::to_string(point_index));
  }

  return node[field_name].as<double>();
}

}  // namespace

void OrigincarGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // planner_server 加载插件时调用 configure。这里读取参数、加载路线，
  // 同时创建一个 NavfnPlanner 作为非预设路线或配置错误时的 fallback。
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock lifecycle node in OrigincarGlobalPlanner");
  }

  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();
  fallback_planner_name_ = name_ + "_fallback_navfn";

  loadParameters();

  // fallback planner 使用独立参数命名空间：GridBased_fallback_navfn。
  // 这样自定义路线规划器和 Navfn 的参数不会互相覆盖。
  nav2_util::declare_parameter_if_not_declared(
    node_, fallback_planner_name_ + ".tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, fallback_planner_name_ + ".use_astar", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, fallback_planner_name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, fallback_planner_name_ + ".use_final_approach_orientation",
    rclcpp::ParameterValue(false));

  fallback_planner_ = std::make_unique<nav2_navfn_planner::NavfnPlanner>();
  fallback_planner_->configure(parent, fallback_planner_name_, tf_, costmap_ros_);

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s with %zu configured routes and %s fallback",
    name_.c_str(), routes_.size(), fallback_navfn_plugin_.c_str());
}

void OrigincarGlobalPlanner::cleanup()
{
  // 生命周期 cleanup 时释放 fallback planner，并清空已解析的路线缓存。
  if (fallback_planner_) {
    fallback_planner_->cleanup();
    fallback_planner_.reset();
  }

  routes_.clear();
  RCLCPP_INFO(node_->get_logger(), "Cleaned up %s", name_.c_str());
}

void OrigincarGlobalPlanner::activate()
{
  // 本插件本身没有 publisher/timer，activate 主要转发生命周期给 fallback planner。
  if (fallback_planner_) {
    fallback_planner_->activate();
  }
  RCLCPP_INFO(node_->get_logger(), "Activated %s", name_.c_str());
}

void OrigincarGlobalPlanner::deactivate()
{
  if (fallback_planner_) {
    fallback_planner_->deactivate();
  }
  RCLCPP_INFO(node_->get_logger(), "Deactivated %s", name_.c_str());
}

nav_msgs::msg::Path OrigincarGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // 目标点 z=-1 时使用动态倒车过渡路径；z=1/2/3/4 时选择预设路线；
  // z=0 或非法值时走 Navfn fallback。
  if (isDynamicReverseGoal(goal)) {
    return createDynamicReversePlan(start, goal);
  }

  const int route_id = routeIdFromGoal(goal);
  if (route_id == 0) {
    return createFallbackPlan(start, goal);
  }

  const auto route_it = routes_.find(route_id);
  if (route_it == routes_.end()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Route id %d is not configured, using fallback Navfn planner", route_id);
    return createFallbackPlan(start, goal);
  }

  if (route_it->second.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Route id %d has no points", route_id);
    return nav_msgs::msg::Path();
  }

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  const size_t first_route_point_index =
    firstConfiguredPointIndex(start, route_it->second, route_id);
  waypoints.reserve(route_it->second.size() - first_route_point_index + 1);
  // 把当前 start 插到预设路线最前面，保证路径从机器人当前位置开始，
  // 而不是强行从 YAML 的第一个固定点开始。
  waypoints.push_back(start);
  waypoints.front().header.frame_id = global_frame_;
  waypoints.front().header.stamp = node_->now();

  for (size_t i = first_route_point_index; i < route_it->second.size(); ++i) {
    auto waypoint = route_it->second[i];
    waypoint.header.frame_id = global_frame_;
    waypoint.header.stamp = node_->now();
    waypoints.push_back(waypoint);
  }

  // YAML 只保存少量关键点；这里生成带朝向的连续平滑路径，供局部控制器跟踪。
  auto path = generateSmoothPath(waypoints);
  // 去掉起点附近已经在车身后方/重叠的点，减少控制器刚启动时选错最近点。
  path = trimPathFromStart(path, start);
  if (path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Route id %d produced an empty path", route_id);
    return nav_msgs::msg::Path();
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Created route %d global path with %zu poses", route_id, path.poses.size());
  return path;
}

void OrigincarGlobalPlanner::loadParameters()
{
  // 默认路线文件从包安装目录读取；参数 routes_file 为空时也回退到该默认文件。
  const std::string default_routes_file =
    ament_index_cpp::get_package_share_directory("origincar_navigation") + "/config/global_routes.yaml";

  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("route_ids"),
    rclcpp::ParameterValue(std::vector<std::string>{"1", "2", "3", "4"}));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("routes_file"), rclcpp::ParameterValue(default_routes_file));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("tension"), rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("min_interpolation_steps"), rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("path_resolution_factor"), rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("trim_distance"), rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("route_start_skip_distance"), rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("transition_turn_radii"),
    rclcpp::ParameterValue(std::vector<double>{0.40, 0.55}));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("transition_reverse_prep_distances"),
    rclcpp::ParameterValue(std::vector<double>{0.0, 0.40, 0.70}));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("transition_sample_interval"), rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("fallback_navfn_plugin"),
    rclcpp::ParameterValue(std::string("nav2_navfn_planner/NavfnPlanner")));

  node_->get_parameter(scopedParam("tension"), tension_);
  node_->get_parameter(scopedParam("routes_file"), routes_file_);
  node_->get_parameter(scopedParam("min_interpolation_steps"), min_interpolation_steps_);
  node_->get_parameter(scopedParam("path_resolution_factor"), path_resolution_factor_);
  node_->get_parameter(scopedParam("trim_distance"), trim_distance_);
  node_->get_parameter(scopedParam("route_start_skip_distance"), route_start_skip_distance_);
  node_->get_parameter(scopedParam("transition_turn_radii"), transition_turn_radii_);
  node_->get_parameter(
    scopedParam("transition_reverse_prep_distances"), transition_reverse_prep_distances_);
  node_->get_parameter(scopedParam("transition_sample_interval"), transition_sample_interval_);
  node_->get_parameter(scopedParam("fallback_navfn_plugin"), fallback_navfn_plugin_);
  if (routes_file_.empty()) {
    routes_file_ = default_routes_file;
  }
  if (tension_ <= 0.0) {
    // 以下参数保护是为了避免插值步数为 0、张力为负等导致路径生成异常。
    RCLCPP_WARN(node_->get_logger(), "tension must be positive, reset to 1.5");
    tension_ = 1.5;
  }
  if (min_interpolation_steps_ < 2) {
    RCLCPP_WARN(node_->get_logger(), "min_interpolation_steps must be at least 2, reset to 10");
    min_interpolation_steps_ = 10;
  }
  if (path_resolution_factor_ <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "path_resolution_factor must be positive, reset to 10.0");
    path_resolution_factor_ = 10.0;
  }
  if (trim_distance_ < 0.0) {
    RCLCPP_WARN(node_->get_logger(), "trim_distance cannot be negative, reset to 0.15");
    trim_distance_ = 0.15;
  }
  if (route_start_skip_distance_ < 0.0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "route_start_skip_distance cannot be negative, reset to 0.3");
    route_start_skip_distance_ = 0.3;
  }
  transition_turn_radii_.erase(
    std::remove_if(
      transition_turn_radii_.begin(), transition_turn_radii_.end(),
      [](double radius) {return radius <= 0.05;}),
    transition_turn_radii_.end());
  if (transition_turn_radii_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "transition_turn_radii is empty, reset to [0.40, 0.55]");
    transition_turn_radii_ = {0.40, 0.55};
  }
  transition_reverse_prep_distances_.erase(
    std::remove_if(
      transition_reverse_prep_distances_.begin(), transition_reverse_prep_distances_.end(),
      [](double distance) {return distance < 0.0;}),
    transition_reverse_prep_distances_.end());
  if (transition_reverse_prep_distances_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "transition_reverse_prep_distances is empty, reset to [0.0, 0.40, 0.70]");
    transition_reverse_prep_distances_ = {0.0, 0.40, 0.70};
  }
  if (transition_sample_interval_ <= 0.005) {
    RCLCPP_WARN(node_->get_logger(), "transition_sample_interval too small, reset to 0.03");
    transition_sample_interval_ = 0.03;
  }

  if (!loadRoutes()) {
    RCLCPP_WARN(node_->get_logger(), "No valid configured routes were loaded");
  }
}

bool OrigincarGlobalPlanner::loadRoutes()
{
  // route_ids 控制从 YAML 中加载哪些 route_N，便于临时只启用部分路线。
  std::vector<std::string> route_ids;
  node_->get_parameter(scopedParam("route_ids"), route_ids);

  routes_.clear();
  if (routes_file_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Route config file path is empty");
    return false;
  }

  YAML::Node config;
  try {
    // YAML::LoadFile 会在文件不存在或格式错误时抛异常。
    config = YAML::LoadFile(routes_file_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to load route config file '%s': %s",
      routes_file_.c_str(), ex.what());
    return false;
  }

  for (const auto & id_text : route_ids) {
    int route_id = 0;
    try {
      route_id = std::stoi(id_text);
    } catch (const std::exception &) {
      RCLCPP_WARN(node_->get_logger(), "Invalid route id '%s'", id_text.c_str());
      continue;
    }

    const auto route_name = "route_" + std::to_string(route_id);
    if (parseRoute(route_id, route_name, config)) {
      RCLCPP_INFO(
        node_->get_logger(), "Loaded route %d with %zu configured points",
        route_id, routes_[route_id].size());
    }
  }

  return !routes_.empty();
}

bool OrigincarGlobalPlanner::parseRoute(
  int route_id,
  const std::string & route_name,
  const YAML::Node & config)
{
  // 每条路线至少要有一个关键点；单点路线会退化成 start -> 该点。
  const YAML::Node route_node = config["routes"][route_name];
  if (!route_node || !route_node["points"] || !route_node["points"].IsSequence()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Route %d is missing '%s.points' sequence in %s",
      route_id, route_name.c_str(), routes_file_.c_str());
    return false;
  }

  const YAML::Node points_node = route_node["points"];
  if (points_node.size() < 1) {
    RCLCPP_WARN(node_->get_logger(), "Route %d must contain at least 1 point", route_id);
    return false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> points;
  points.reserve(points_node.size());
  try {
    for (size_t i = 0; i < points_node.size(); ++i) {
      points.push_back(parseRoutePoint(points_node[i], route_name, i));
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to parse route %d from %s: %s",
      route_id, routes_file_.c_str(), ex.what());
    return false;
  }

  routes_[route_id] = std::move(points);
  return true;
}

size_t OrigincarGlobalPlanner::firstConfiguredPointIndex(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<geometry_msgs::msg::PoseStamped> & route_points,
  int route_id) const
{
  if (route_points.size() < 2 || route_start_skip_distance_ <= 0.0) {
    return 0;
  }

  const Vector2 start_xy{start.pose.position.x, start.pose.position.y};
  const Vector2 first_route_point_xy{
    route_points.front().pose.position.x, route_points.front().pose.position.y};
  const double start_to_first_point_distance = distance(start_xy, first_route_point_xy);
  if (start_to_first_point_distance > route_start_skip_distance_) {
    return 0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Route %d skips first configured point: start distance %.3f m <= %.3f m and %zu points remain",
    route_id, start_to_first_point_distance, route_start_skip_distance_, route_points.size() - 1);
  return 1;
}

geometry_msgs::msg::PoseStamped OrigincarGlobalPlanner::parseRoutePoint(
  const YAML::Node & point_node,
  const std::string & route_name,
  size_t point_index) const
{
  // 路线点统一转换成 PoseStamped，frame 在后续 createPlan 中会刷新为当前 global_frame。
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = node_->now();
  pose.pose.position.x = readRequiredDouble(point_node, "x", route_name, point_index);
  pose.pose.position.y = readRequiredDouble(point_node, "y", route_name, point_index);
  pose.pose.position.z = 0.0;

  if (point_node["orientation"]) {
    // 支持直接粘贴 RViz 输出的 quaternion。
    const auto orientation = point_node["orientation"];
    pose.pose.orientation.x = readRequiredDouble(orientation, "x", route_name, point_index);
    pose.pose.orientation.y = readRequiredDouble(orientation, "y", route_name, point_index);
    pose.pose.orientation.z = readRequiredDouble(orientation, "z", route_name, point_index);
    pose.pose.orientation.w = readRequiredDouble(orientation, "w", route_name, point_index);
  } else if (point_node["yaw_deg"]) {
    // 支持更易读的角度制 yaw。
    pose.pose.orientation = quaternionFromYaw(point_node["yaw_deg"].as<double>() * kDegreesToRadians);
  } else if (point_node["yaw"]) {
    // 也支持弧度制 yaw，方便程序生成配置。
    pose.pose.orientation = quaternionFromYaw(point_node["yaw"].as<double>());
  } else {
    throw std::runtime_error(
            "Point " + std::to_string(point_index) + " in " + route_name +
            " must define orientation quaternion, yaw_deg, or yaw");
  }

  return pose;
}

nav_msgs::msg::Path OrigincarGlobalPlanner::generateSmoothPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints) const
{
  // 用 Hermite 曲线连接关键点。相比直线折线，Hermite 能利用每个点的朝向生成更平顺的路径，
  // 对阿克曼车更友好，减少局部控制器面对尖角时的大幅转向。
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  if (waypoints.size() < 2) {
    return path;
  }

  std::vector<geometry_msgs::msg::PoseStamped> extended_points;
  extended_points.reserve(waypoints.size() + 2);

  // 在首尾各添加一个虚拟点，让首段和尾段也能根据端点朝向计算出稳定切线。
  auto virtual_start = waypoints.front();
  double yaw = tf2::getYaw(virtual_start.pose.orientation);
  virtual_start.pose.position.x -= std::cos(yaw);
  virtual_start.pose.position.y -= std::sin(yaw);
  extended_points.push_back(virtual_start);

  extended_points.insert(extended_points.end(), waypoints.begin(), waypoints.end());

  auto virtual_end = waypoints.back();
  yaw = tf2::getYaw(virtual_end.pose.orientation);
  virtual_end.pose.position.x += std::cos(yaw);
  virtual_end.pose.position.y += std::sin(yaw);
  extended_points.push_back(virtual_end);

  for (size_t i = 1; i + 2 < extended_points.size(); ++i) {
    // 每次用 prev/start/end/next 四个点计算 start->end 这一段的受控切线。
    appendSmoothSegment(
      extended_points[i - 1], extended_points[i], extended_points[i + 1],
      extended_points[i + 2], path.poses);
  }

  for (auto & pose : path.poses) {
    // 统一刷新 header，避免局部控制器在 TF 查询时碰到旧时间戳。
    pose.header.frame_id = global_frame_;
    pose.header.stamp = path.header.stamp;
    pose.pose.position.z = 0.0;
  }

  return path;
}

void OrigincarGlobalPlanner::appendSmoothSegment(
  const geometry_msgs::msg::PoseStamped & prev,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  const geometry_msgs::msg::PoseStamped & next,
  std::vector<geometry_msgs::msg::PoseStamped> & poses) const
{
  // 取四个邻近点的位置和 start/end 的显式朝向，生成一段三次 Hermite 曲线。
  const Vector2 p_prev{prev.pose.position.x, prev.pose.position.y};
  const Vector2 p_start{start.pose.position.x, start.pose.position.y};
  const Vector2 p_end{end.pose.position.x, end.pose.position.y};
  const Vector2 p_next{next.pose.position.x, next.pose.position.y};

  const double yaw_start = tf2::getYaw(start.pose.orientation);
  const double yaw_end = tf2::getYaw(end.pose.orientation);
  const Vector2 tangent_start{std::cos(yaw_start), std::sin(yaw_start)};
  const Vector2 tangent_end{std::cos(yaw_end), std::sin(yaw_end)};

  const auto controlled_tangent = [this](
      const Vector2 & point_prev,
      const Vector2 & point_current,
      const Vector2 & point_next,
      const Vector2 & current_tangent) {
      const Vector2 in_vec = normalized(point_current - point_prev, current_tangent);
      const Vector2 out_vec = normalized(point_next - point_current, current_tangent);

      // 默认切线取入射方向和出射方向的角平分方向；如果出现折返，则优先尊重配置点朝向。
      Vector2 base_tangent = normalized(in_vec + out_vec, current_tangent);
      if (dot(in_vec, out_vec) < 0.0) {
        base_tangent = current_tangent;
      }

      const double turn_angle = clampedAcos(dot(in_vec, out_vec));
      double boost_factor = 1.0;
      if (turn_angle > M_PI / 6.0) {
        // 大转角处增强配置朝向的影响，避免曲线为了追求平滑而切到路线外侧太多。
        const double ratio = (turn_angle - M_PI / 6.0) / (M_PI / 2.0);
        boost_factor = 1.0 + ratio * 2.0;
      }

      return normalized(base_tangent + current_tangent * (tension_ * boost_factor), current_tangent);
    };

  Vector2 m1 = controlled_tangent(p_prev, p_start, p_end, tangent_start);
  Vector2 m2 = controlled_tangent(p_start, p_end, p_next, tangent_end);

  const double segment_length = norm(p_end - p_start);
  if (segment_length <= std::numeric_limits<double>::epsilon()) {
    return;
  }

  m1 = m1 * (segment_length * tension_);
  m2 = m2 * (segment_length * tension_);

  const Vector2 segment_dir = normalized(p_end - p_start);
  if (dot(segment_dir, tangent_start) < 0.0) {
    // 如果配置朝向与实际段方向相反，缩短该端切线，避免生成回头弯。
    m1 = tangent_start * (segment_length * 0.5);
  }
  if (dot(segment_dir, tangent_end) < 0.0) {
    m2 = tangent_end * (segment_length * 0.5);
  }

  const int steps = std::max(
    min_interpolation_steps_,
    static_cast<int>(std::ceil(segment_length * path_resolution_factor_)));

  for (int i = 0; i <= steps; ++i) {
    if (!poses.empty() && i == 0) {
      // 相邻段共享端点，跳过重复点，避免控制器看到零长度段。
      continue;
    }

    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double t_sq = t * t;
    const double t_cu = t_sq * t;

    const double h00 = 2.0 * t_cu - 3.0 * t_sq + 1.0;
    const double h10 = t_cu - 2.0 * t_sq + t;
    const double h01 = -2.0 * t_cu + 3.0 * t_sq;
    const double h11 = t_cu - t_sq;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;
    pose.pose.position.x = h00 * p_start.x + h10 * m1.x + h01 * p_end.x + h11 * m2.x;
    pose.pose.position.y = h00 * p_start.y + h10 * m1.y + h01 * p_end.y + h11 * m2.y;
    pose.pose.position.z = 0.0;

    const Vector2 derivative{
      (6.0 * t_sq - 6.0 * t) * p_start.x + (3.0 * t_sq - 4.0 * t + 1.0) * m1.x +
        (-6.0 * t_sq + 6.0 * t) * p_end.x + (3.0 * t_sq - 2.0 * t) * m2.x,
      (6.0 * t_sq - 6.0 * t) * p_start.y + (3.0 * t_sq - 4.0 * t + 1.0) * m1.y +
        (-6.0 * t_sq + 6.0 * t) * p_end.y + (3.0 * t_sq - 2.0 * t) * m2.y};

    // 用曲线导数作为路径点朝向，使控制器接近终点或调试显示时方向连续。
    pose.pose.orientation = quaternionFromYaw(std::atan2(derivative.y, derivative.x));
    poses.push_back(pose);
  }
}

nav_msgs::msg::Path OrigincarGlobalPlanner::trimPathFromStart(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start) const
{
  // 从路径尾部向前找离 start 最近的一段，保留它之后的路径。
  // 这样可以删除机器人身后或与当前位置重叠的插值点。
  if (path.poses.empty() || trim_distance_ <= 0.0) {
    return path;
  }

  const double trim_distance_sq = trim_distance_ * trim_distance_;
  auto found = path.poses.rend();
  for (auto it = path.poses.rbegin(); it != path.poses.rend(); ++it) {
    const double dx = it->pose.position.x - start.pose.position.x;
    const double dy = it->pose.position.y - start.pose.position.y;
    if (dx * dx + dy * dy < trim_distance_sq) {
      found = it;
      break;
    }
  }

  if (found == path.poses.rend()) {
    return path;
  }

  auto keep_begin = (found + 1).base();
  nav_msgs::msg::Path trimmed;
  trimmed.header = path.header;
  trimmed.poses.assign(keep_begin, path.poses.end());
  return trimmed;
}

nav_msgs::msg::Path OrigincarGlobalPlanner::createFallbackPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // 非 route 目标仍然交给 NavfnPlanner，保证普通 2D Goal 不会因为未编码路线而失效。
  if (!fallback_planner_) {
    RCLCPP_ERROR(node_->get_logger(), "Fallback Navfn planner is not configured");
    return nav_msgs::msg::Path();
  }

  RCLCPP_DEBUG(node_->get_logger(), "Using fallback Navfn planner");
  return fallback_planner_->createPlan(start, goal);
}

nav_msgs::msg::Path OrigincarGlobalPlanner::createDynamicReversePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const Vector2 start_xy{start.pose.position.x, start.pose.position.y};
  const Vector2 goal_xy{goal.pose.position.x, goal.pose.position.y};
  (void)start_xy;
  (void)goal_xy;

  for (const double prep_distance : transition_reverse_prep_distances_) {
    for (const double turn_radius : transition_turn_radii_) {
      auto candidate = createReverseTurnCandidate(start, goal, prep_distance, turn_radius);
      if (candidate.poses.empty()) {
        continue;
      }
      if (!isPathCollisionFree(candidate)) {
        continue;
      }

      markDynamicTransitionPath(candidate);
      RCLCPP_INFO(
        node_->get_logger(),
        "Created reverse-turn transition path with %zu poses, prep=%.2f m, radius=%.2f m",
        candidate.poses.size(), prep_distance, turn_radius);
      return candidate;
    }
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "No collision-free dynamic transition candidate found, using fallback smooth transition");

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.reserve(2);

  auto start_pose = start;
  start_pose.header.frame_id = global_frame_;
  start_pose.header.stamp = node_->now();
  start_pose.pose.position.z = 0.0;
  start_pose.pose.orientation =
    quaternionFromYaw(angles::normalize_angle(tf2::getYaw(start.pose.orientation) + M_PI));
  waypoints.push_back(start_pose);

  auto goal_pose = goal;
  goal_pose.header.frame_id = global_frame_;
  goal_pose.header.stamp = node_->now();
  goal_pose.pose.position.z = 0.0;
  const auto parking_orientation = goal_pose.pose.orientation;
  goal_pose.pose.orientation =
    quaternionFromYaw(angles::normalize_angle(tf2::getYaw(goal.pose.orientation) + M_PI));
  waypoints.push_back(goal_pose);

  auto path = generateSmoothPath(waypoints);
  path = trimPathFromStart(path, start);
  if (path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Dynamic reverse transition produced an empty path");
    return nav_msgs::msg::Path();
  }
  path.poses.back().pose.orientation = parking_orientation;
  markDynamicTransitionPath(path);

  RCLCPP_INFO(
    node_->get_logger(),
    "Created dynamic reverse transition path with %zu poses", path.poses.size());
  return path;
}

nav_msgs::msg::Path OrigincarGlobalPlanner::createForwardTransitionPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double target_yaw) const
{
  auto start_pose = start;
  start_pose.header.frame_id = global_frame_;
  start_pose.header.stamp = node_->now();
  start_pose.pose.position.z = 0.0;

  auto goal_pose = goal;
  goal_pose.header.frame_id = global_frame_;
  goal_pose.header.stamp = node_->now();
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation = quaternionFromYaw(target_yaw);

  auto path = generateSmoothPath({start_pose, goal_pose});
  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal_pose.pose.orientation;
  }
  return path;
}

nav_msgs::msg::Path OrigincarGlobalPlanner::createReverseTurnCandidate(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double prep_distance,
  double turn_radius) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  auto current = start;
  current.header = path.header;
  current.pose.position.z = 0.0;
  path.poses.push_back(current);

  appendReverseStraight(path, current, prep_distance);
  current = path.poses.back();

  const double target_yaw = std::atan2(
    goal.pose.position.y - current.pose.position.y,
    goal.pose.position.x - current.pose.position.x);
  current = appendReverseArc(path, current, target_yaw, turn_radius);

  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = quaternionFromYaw(target_yaw);
  }
  return path;
}

void OrigincarGlobalPlanner::appendReverseStraight(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  double distance) const
{
  if (distance <= 0.0) {
    return;
  }

  const double yaw = tf2::getYaw(start.pose.orientation);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / transition_sample_interval_)));
  for (int i = 1; i <= steps; ++i) {
    const double s = distance * static_cast<double>(i) / static_cast<double>(steps);
    auto pose = start;
    pose.header = path.header;
    pose.pose.position.x = start.pose.position.x - s * std::cos(yaw);
    pose.pose.position.y = start.pose.position.y - s * std::sin(yaw);
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);
  }
}

geometry_msgs::msg::PoseStamped OrigincarGlobalPlanner::appendReverseArc(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  double target_yaw,
  double turn_radius) const
{
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double yaw_delta = angles::shortest_angular_distance(start_yaw, target_yaw);
  if (std::fabs(yaw_delta) < 1.0e-3 || turn_radius <= 0.0) {
    return start;
  }

  // 倒车时，路径曲率符号与车头 yaw 变化方向相反。
  const double curvature = yaw_delta > 0.0 ? -1.0 / turn_radius : 1.0 / turn_radius;
  const double arc_length = std::fabs(yaw_delta) * turn_radius;
  const int steps = std::max(2, static_cast<int>(std::ceil(arc_length / transition_sample_interval_)));

  geometry_msgs::msg::PoseStamped last = start;
  for (int i = 1; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(steps);
    const double yaw = angles::normalize_angle(start_yaw + yaw_delta * ratio);

    auto pose = start;
    pose.header = path.header;
    pose.pose.position.x =
      start.pose.position.x + (std::sin(yaw) - std::sin(start_yaw)) / curvature;
    pose.pose.position.y =
      start.pose.position.y - (std::cos(yaw) - std::cos(start_yaw)) / curvature;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = quaternionFromYaw(yaw);
    path.poses.push_back(pose);
    last = pose;
  }

  return last;
}

void OrigincarGlobalPlanner::appendPathSkippingFirst(
  nav_msgs::msg::Path & path,
  const nav_msgs::msg::Path & suffix) const
{
  if (suffix.poses.empty()) {
    return;
  }

  for (size_t i = 1; i < suffix.poses.size(); ++i) {
    auto pose = suffix.poses[i];
    pose.header = path.header;
    path.poses.push_back(pose);
  }
}

void OrigincarGlobalPlanner::markDynamicTransitionPath(nav_msgs::msg::Path & path) const
{
  for (auto & pose : path.poses) {
    pose.header.frame_id = global_frame_;
    pose.header.stamp = path.header.stamp;
    pose.pose.position.z = kDynamicReverseGoalCode;
  }
}

bool OrigincarGlobalPlanner::isPathCollisionFree(const nav_msgs::msg::Path & path) const
{
  if (!costmap_ros_) {
    return true;
  }
  const auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return true;
  }

  for (const auto & pose : path.poses) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      return false;
    }
    if (costmap->getCost(mx, my) >= kTransitionBlockedCost) {
      return false;
    }
  }
  return true;
}

bool OrigincarGlobalPlanner::isDynamicReverseGoal(
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return std::fabs(goal.pose.position.z - kDynamicReverseGoalCode) <= kGoalRouteTolerance;
}

int OrigincarGlobalPlanner::routeIdFromGoal(const geometry_msgs::msg::PoseStamped & goal) const
{
  // 约定：goal.pose.position.z = N 表示选择 route_N。
  // RViz 普通目标 z 通常为 0，因此会自然走 fallback。
  const double encoded_id = goal.pose.position.z;
  const auto rounded_id = static_cast<int>(std::lround(encoded_id));
  if (std::fabs(encoded_id - static_cast<double>(rounded_id)) > kGoalRouteTolerance) {
    return 0;
  }

  return rounded_id >= 1 ? rounded_id : 0;
}

std::string OrigincarGlobalPlanner::scopedParam(const std::string & param_name) const
{
  return name_ + "." + param_name;
}

}  // namespace origincar_navigation

PLUGINLIB_EXPORT_CLASS(origincar_navigation::OrigincarGlobalPlanner, nav2_core::GlobalPlanner)
