#include "origincar_navigation/origincar_ackermann_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace origincar_navigation
{
namespace
{

// 下面这些常量刻意保留 ROS1 参考实现中的数值写法，而不是换成更“现代”的
// M_PI 换算。参考文件里转向角先乘 57.3 变成度，输出 /cmd_vel 时再乘
// 0.0174533 转回弧度；这里照搬这个单位往返，避免出现细小但难追的行为差异。
constexpr double kReferenceRadiansToDegrees = 57.3;
constexpr double kReferenceDegreesToRadians = 0.0174533;
constexpr double kReferencePoseAdjustDistance = 0.25;
constexpr double kReferenceOrientationKp = 1.5;
constexpr double kReferenceGoalDecelDistance = 0.3;
constexpr double kReferenceMinApproachSpeed = 0.05;
constexpr double kReferenceYawSlowdownThreshold = 0.2;
constexpr double kReferenceMinTurnSpeedFactor = 0.2;

// 由 yaw 角生成路径点朝向。控制器在终点延长虚拟前视点时会用到，
// 这样虚拟点既有位置，也有和路径末端一致的朝向信息。
geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

double poseDistance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  const double dx = lhs.pose.position.x - rhs.pose.position.x;
  const double dy = lhs.pose.position.y - rhs.pose.position.y;
  return std::hypot(dx, dy);
}

}  // namespace

void OrigincarAckermannPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Nav2 控制器插件运行在 controller_server 生命周期节点内部。
  // configure 阶段只做资源申请和参数读取，不输出控制指令。
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock lifecycle node in OrigincarAckermannPurePursuitController");
  }

  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  robot_base_frame_ = costmap_ros_->getBaseFrameID();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  // 控制算法参数挂在 PurePursuit 这个控制器命名空间下，例如 PurePursuit.wheelbase。
  // 路径提供器和局部避障参数使用共享命名空间，便于两个控制器共用同一套路由配置。
  loadParameters();
  tracking_route_provider_ = createTrackingRouteProvider(tracking_route_provider_plugin_);
  tracking_route_provider_->configure(
    node_, "tracking_route_provider", name_, tf_, costmap_ros_, transform_tolerance_);

  // 这些话题用于 RViz 调试：local_plan 看当前被跟踪的路径，
  // target_pose 看纯追踪前视点，goal_reached 看控制器内部到达判断。
  local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(name_ + "/local_plan", 1);
  target_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/target_pose", 1);
  goal_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(name_ + "/goal_reached", 1);

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s pure pursuit controller: wheelbase=%.3f, max_angular_=%.3f deg",
    name_.c_str(), wheelbase_, max_angular_);
}

void OrigincarAckermannPurePursuitController::cleanup()
{
  // 生命周期 cleanup 阶段释放 publisher 和路径缓存，避免重新 configure 时保留旧状态。
  local_plan_pub_.reset();
  target_pose_pub_.reset();
  goal_reached_pub_.reset();
  if (tracking_route_provider_) {
    tracking_route_provider_->cleanup();
  }
  base_plan_ = nav_msgs::msg::Path();
  RCLCPP_INFO(node_->get_logger(), "Cleaned up %s", name_.c_str());
}

void OrigincarAckermannPurePursuitController::activate()
{
  local_plan_pub_->on_activate();
  target_pose_pub_->on_activate();
  goal_reached_pub_->on_activate();
  if (tracking_route_provider_) {
    tracking_route_provider_->activate();
  }
  RCLCPP_INFO(node_->get_logger(), "Activated %s", name_.c_str());
}

void OrigincarAckermannPurePursuitController::deactivate()
{
  local_plan_pub_->on_deactivate();
  target_pose_pub_->on_deactivate();
  goal_reached_pub_->on_deactivate();
  if (tracking_route_provider_) {
    tracking_route_provider_->deactivate();
  }
  RCLCPP_INFO(node_->get_logger(), "Deactivated %s", name_.c_str());
}

void OrigincarAckermannPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  // Nav2 每次规划完成后会调用 setPlan。控制器只保存全局基准路径；
  // 实际跟踪路径由 TrackingRouteProvider 维护，便于局部避障和控制算法解耦。
  base_plan_ = path;
  if (tracking_route_provider_) {
    tracking_route_provider_->resetBasePlan(path);
  }
  last_goal_reached_ = false;
}

geometry_msgs::msg::TwistStamped OrigincarAckermannPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = robot_base_frame_;

  if (base_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Origincar pure pursuit received an empty global plan");
  }

  geometry_msgs::msg::PoseStamped robot_pose = pose;
  if (robot_pose.header.frame_id.empty()) {
    robot_pose.header.frame_id = base_plan_.header.frame_id;
  }
  if (robot_pose.header.frame_id != global_frame_) {
    geometry_msgs::msg::PoseStamped robot_pose_in_costmap_frame;
    if (!transformPose(robot_pose, global_frame_, robot_pose_in_costmap_frame)) {
      throw nav2_core::PlannerException("Failed to transform robot pose to local costmap frame");
    }
    robot_pose = robot_pose_in_costmap_frame;
  }

  // 复用 controller_server 传入的 GoalChecker，保持 action success 条件与
  // 控制器立即停车条件完全一致，同时仍保留“到点立刻输出 0 速度”的保护。
  geometry_msgs::msg::PoseStamped goal_pose = base_plan_.poses.back();
  if (!transformPose(goal_pose, robot_pose.header.frame_id, goal_pose)) {
    throw nav2_core::PlannerException("Failed to transform goal pose for goal checking");
  }
  if (goal_checker != nullptr &&
    goal_checker->isGoalReached(robot_pose.pose, goal_pose.pose, velocity))
  {
    current_velocity_ = 0.0;
    reverse_mode_ = false;
    publishDebug({}, base_plan_.poses.back(), true);
    return cmd_vel;
  }

  const auto tracking_path = tracking_route_provider_ ?
    tracking_route_provider_->getTrackingPath(robot_pose) : base_plan_;
  if (tracking_path.poses.empty()) {
    throw nav2_core::PlannerException("Tracking route provider returned an empty path");
  }

  // provider 对外始终返回 map 路径。当前 local_costmap 也使用 map，
  // 因此避障路径不会被 odom 漂移固化到长期 tracking path 中。
  const auto plan = transformedPlan(tracking_path, global_frame_);
  if (plan.empty()) {
    throw nav2_core::PlannerException("Failed to transform tracking path for pure pursuit");
  }

  // ROS1 参考文件在 computeVelocityCommands() 中先用成员变量 global_plan_ 作为跟踪路径。
  // ROS2/Nav2 适配点：这里保留现有 TrackingRouteProvider，provider 可能会把原始全局路径
  // 替换成局部避障后的 tracking_path；从这一行开始，后面的纯追踪步骤只把 tracking_plan
  // 当作参考文件里的 global_plan_ 来用，不再加入当前 ROS2 版本原有的额外控制增强。
  const auto & tracking_plan = plan;

  // 对应参考文件：current_velocity_ 保存当前线速度，然后 checkNeedReverse(robot_pose)。
  // Nav2 会把当前速度作为 computeVelocityCommands() 的入参传进来，因此这里直接取
  // velocity.linear.x，而不是像 ROS1 节点那样额外订阅里程计。
  current_velocity_ = velocity.linear.x;
  reverse_mode_ = shouldReverse(tracking_plan, robot_pose);

  // 对应参考文件：getAdaptiveLookaheadDistance(current_velocity_)。
  // 倒车时仍沿用动态前视距离，但不再允许前视距离短到 0.1m 量级；
  // 否则直线倒车时一点点横向误差都会被曲率公式放大成满角。
  const double lookahead_distance = adaptiveLookaheadDistance(current_velocity_);

  // 对应参考文件：getLookaheadPoint(robot_pose, lookahead_dist, true)。
  // 这里会先找最近路径点，再向后找第一个满足前视距离的点；如果路径末端不够长，
  // 则按终点 yaw 创建虚拟延长点，用于终点附近的姿态校正。
  const auto target_pose = getLookaheadPoint(tracking_plan, robot_pose, lookahead_distance, true);

  // 对应参考文件：calculateSteering(target_pose, robot_pose)。
  // 这个返回值是“度制前轮等效转角”，不是最终发给下位机的角速度。
  // 保持度制是为了完整复刻参考实现中 angular_z 的语义；最终输出时再乘
  // 0.0174533 转回弧度，并按自行车模型换算成标准 /cmd_vel.angular.z。
  const double steering_degrees = steeringAngle(tracking_plan, target_pose, robot_pose);

  // 对应参考文件：acalculateVel(target_pose, angular_z)。
  // 返回值始终是正的速度幅值；倒车符号在下面输出 cmd_vel 时再统一处理。
  const double commanded_linear_speed =
    targetLinearVelocity(robot_pose, tracking_plan.back(), steering_degrees);
  cmd_vel.twist.linear.x = reverse_mode_ ? -commanded_linear_speed : commanded_linear_speed;
  const double steering_radians = steering_degrees * kReferenceDegreesToRadians;
  // 下位机当前直接接收 /cmd_vel.angular.z 作为角速度，不再经过
  // cmd_vel_to_ackermann_drive.py 转换。这里按自行车模型输出标准 yaw rate：
  //   omega = signed_v * tan(delta) / wheelbase
  // 倒车时 signed_v 为负，因此角速度方向会自然随倒车运动学翻转。
  cmd_vel.twist.angular.z =
    wheelbase_ > 1e-6 ?
    (cmd_vel.twist.linear.x * std::tan(steering_radians))*0.8 / wheelbase_ : 0.0;

  current_velocity_ = cmd_vel.twist.linear.x;
  publishDebug(tracking_plan, target_pose, false);
  return cmd_vel;
}

void OrigincarAckermannPurePursuitController::setSpeedLimit(
  const double & /*speed_limit*/,
  const bool & /*percentage*/)
{
  // 本项目不使用 Nav2 speed filter 动态限速。接口由 nav2_core::Controller 要求保留。
}

std::string OrigincarAckermannPurePursuitController::scopedParam(
  const std::string & param_name) const
{
  return name_ + "." + param_name;
}

void OrigincarAckermannPurePursuitController::loadParameters()
{
  // declare_parameter_if_not_declared 允许参数文件覆盖默认值，同时避免重复声明导致异常。
  // 本段参数名按 /root/digua/doc/pure_pursuit_planner.cpp 的 initialize() 尽量复刻。
  // ROS2 适配点：wheelbase 使用实车下位机角速度换算对应的 0.143m；max_angular_
  // 使用参考文件的“度制转角”语义，但默认值由旧 max_steering_angle=0.65rad
  // 换算得到，避免保持现有 /cmd_vel 输出格式时把车辆转向能力意外压到 1 度。
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("min_lookahead_distance"), rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("max_lookahead_distance"), rclcpp::ParameterValue(2.00));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("lookahead_ratio"), rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("max_linear_velocity"), rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("max_angular_"), rclcpp::ParameterValue(37.24));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("goal_tolerance"), rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("decel_angle_threshold"), rclcpp::ParameterValue(30.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("decel_velocity_threshold"), rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("decel_ratio"), rclcpp::ParameterValue(0.80));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("max_angular_vel"), rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("wheelbase"), rclcpp::ParameterValue(0.143));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("reverse_lookahead_ratio"), rclcpp::ParameterValue(1.00));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("max_reverse_velocity"), rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("reverse_check_distance"), rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("transform_tolerance"), rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, "tracking_route_provider.plugin",
    rclcpp::ParameterValue("origincar_navigation/SegmentBypassTrackingRouteProvider"));

  node_->get_parameter(scopedParam("min_lookahead_distance"), min_lookahead_distance_);
  node_->get_parameter(scopedParam("max_lookahead_distance"), max_lookahead_distance_);
  node_->get_parameter(scopedParam("lookahead_ratio"), lookahead_ratio_);
  node_->get_parameter(scopedParam("max_linear_velocity"), max_linear_velocity_);
  node_->get_parameter(scopedParam("max_angular_"), max_angular_);
  node_->get_parameter(scopedParam("goal_tolerance"), goal_tolerance_);
  node_->get_parameter(scopedParam("decel_angle_threshold"), decel_angle_threshold_);
  node_->get_parameter(scopedParam("decel_velocity_threshold"), decel_velocity_threshold_);
  node_->get_parameter(scopedParam("decel_ratio"), decel_ratio_);
  node_->get_parameter(scopedParam("max_angular_vel"), max_angular_vel_);
  node_->get_parameter(scopedParam("wheelbase"), wheelbase_);
  node_->get_parameter(scopedParam("reverse_lookahead_ratio"), reverse_lookahead_ratio_);
  node_->get_parameter(scopedParam("max_reverse_velocity"), max_reverse_velocity_);
  node_->get_parameter(scopedParam("reverse_check_distance"), reverse_check_distance_);
  node_->get_parameter(scopedParam("transform_tolerance"), transform_tolerance_);
  node_->get_parameter("tracking_route_provider.plugin", tracking_route_provider_plugin_);

  // 参考 ROS1 代码本身没有完整的参数合法性检查。这里仅保留避免除零和负速度的底线保护。
  // 倒车前视距离会在 adaptiveLookaheadDistance() 中保证不低于 min_lookahead_distance_，
  // 避免倒车直线跟踪时因前视距离过短而角速度打满。
  max_lookahead_distance_ = std::max(max_lookahead_distance_, min_lookahead_distance_);
  wheelbase_ = std::max(wheelbase_, 1e-3);
  max_angular_ = std::max(max_angular_, 1e-3);
  max_linear_velocity_ = std::max(max_linear_velocity_, 0.0);
  max_reverse_velocity_ = std::max(max_reverse_velocity_, 0.0);
  max_angular_vel_ = std::max(max_angular_vel_, 0.0);
  decel_velocity_threshold_ = std::max(decel_velocity_threshold_, 0.0);
}

bool OrigincarAckermannPurePursuitController::transformPose(
  const geometry_msgs::msg::PoseStamped & in_pose,
  const std::string & target_frame,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == target_frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    auto pose = in_pose;
    // 全局路径点可能带有规划时刻的旧 stamp。控制器运行一段时间后，
    // 如果按旧时间查 map->odom，容易出现 extrapolation into the past。
    // stamp=0 表示使用最新可用 TF，更适合静态全局路径跟踪。
    pose.header.stamp.sec = 0;
    pose.header.stamp.nanosec = 0;
    out_pose = tf_->transform(
      pose, target_frame, tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Failed to transform pose from %s to %s: %s",
      in_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

std::vector<geometry_msgs::msg::PoseStamped>
OrigincarAckermannPurePursuitController::transformedPlan(
  const nav_msgs::msg::Path & path,
  const std::string & target_frame) const
{
  // 返回一份转换后的路径副本，避免修改 provider 维护的 map 路径。
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  plan.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (!transformPose(pose, target_frame, transformed_pose)) {
      return {};
    }
    plan.push_back(transformed_pose);
  }
  return plan;
}

size_t OrigincarAckermannPurePursuitController::closestPoseIndex(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  // 对应参考文件 getLookaheadPoint() 的第一段：
  //   size_t closest_idx = 0;
  //   double min_dist = std::numeric_limits<double>::max();
  //   for (...) {
  //     double dist = hypot(dx, dy);
  //     if (dist < min_dist) { ... }
  //   }
  //
  // 这里故意使用 hypot 计算真实欧氏距离，而不是当前 ROS2 旧版曾用过的距离平方。
  // 两者在排序上通常等价，但用户要求尽量复刻参考算法，所以连这个细节也按参考写法恢复。
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan.size(); ++i) {
    const double dx = plan[i].pose.position.x - pose.pose.position.x;
    const double dy = plan[i].pose.position.y - pose.pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

geometry_msgs::msg::PoseStamped OrigincarAckermannPurePursuitController::getLookaheadPoint(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead_distance,
  bool allow_goal_extension) const
{
  // const size_t closest_idx = closestPoseIndex(plan, robot_pose);
  const size_t closest_idx = 0;

  // 对应参考文件 getLookaheadPoint() 的第二段：
  // 从“距离机器人最近的路径点”开始向路径末端搜索，找到第一个与机器人当前位置
  // 欧氏距离 >= lookahead_dist 的路径点后立刻返回。注意这里不是沿路径弧长累计，
  // 参考实现就是直接拿每个候选点到机器人当前位置的直线距离做判断。
  for (size_t i = closest_idx; i < plan.size(); ++i) {
    if (poseDistance(plan[i], robot_pose) >= lookahead_distance) {
      return plan[i];
    }
  }

  if (!allow_goal_extension) {
    // 对应参考文件 need_extend == false 的分支：短前视点查找失败时直接返回终点。
    // shouldReverse() 会用这个分支判断 0.2m 前视点是否落在车体后方。
    return plan.back();
  }

  // 对应参考文件 need_extend == true 的分支：如果真实路径点都不满足前视距离，
  // 复制路径终点，后续必要时沿终点朝向创建虚拟延长点。该虚拟点不是写回路径，
  // 只作为本周期纯追踪目标点使用。
  auto virtual_point = plan.back();
  double goal_yaw = tf2::getYaw(virtual_point.pose.orientation);
  if (reverse_mode_) {
    // 参考实现倒车模式下把终点 yaw 翻转 180 度，然后用 while 手动归一化到 [-pi, pi]。
    // 这里不换成 angles::normalize_angle，是为了让边界行为尽量和 ROS1 文件一致。
    goal_yaw += M_PI;
    while (goal_yaw > M_PI) {
      goal_yaw -= 2.0 * M_PI;
    }
    while (goal_yaw < -M_PI) {
      goal_yaw += 2.0 * M_PI;
    }
  }

  // 参考实现计算机器人到终点的直线距离，并额外加 0.1m 校准空间：
  //   extend_distance = lookahead_dist - dist_to_goal + 0.1
  // 只有 extend_distance > 0 时才真正移动虚拟点；否则返回原终点副本。
  const double dist_to_goal = poseDistance(virtual_point, robot_pose);
  const double extend_distance = lookahead_distance - dist_to_goal + 0.10;
  if (extend_distance > 0.0) {
    // 参考实现沿 goal_yaw 方向平移虚拟点，并把虚拟点朝向也设置为 goal_yaw。
    // ROS2 适配点：四元数生成函数换成 tf2::toMsg，但 yaw 数值来源和参考一致。
    virtual_point.pose.position.x += extend_distance * std::cos(goal_yaw);
    virtual_point.pose.position.y += extend_distance * std::sin(goal_yaw);
    virtual_point.pose.orientation = quaternionFromYaw(goal_yaw);
  }

  return virtual_point;
}

double OrigincarAckermannPurePursuitController::adaptiveLookaheadDistance(
  double current_speed) const
{
  // 基本对应参考文件 getAdaptiveLookaheadDistance(current_speed)：
  //   distance = min_lookahead_distance_ + lookahead_ratio_ * fabs(current_speed)
  //   if (is_reverse_mode_) distance *= reverse_lookahead_ratio_
  //   return std::min(distance, max_lookahead_distance_)
  //
  // 实车倒车时如果把 0.30m 前视距离乘到 0.12m，纯追踪曲率会对几厘米横向误差
  // 极其敏感，表现为“应该直着倒却角速度打满”。因此倒车分支保留 ratio 参数，
  // 但最终不允许小于 min_lookahead_distance_。
  double distance = min_lookahead_distance_ + lookahead_ratio_ * std::fabs(current_speed);
  if (reverse_mode_) {
    distance *= reverse_lookahead_ratio_;
    distance = std::max(distance, min_lookahead_distance_);
  }
  return std::min(distance, max_lookahead_distance_);
}

bool OrigincarAckermannPurePursuitController::shouldReverse(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  // 对应参考文件 checkNeedReverse(robot_pose)：
  // 1. getLookaheadPoint(robot_pose, 0.2, false) 取一个很短的检查点。
  // 2. 把检查点转换到 robot_base_frame_。
  // 3. 如果检查点在车体坐标系 x < 0，说明目标点落在车体后方，本周期进入倒车模式。
  //
  // ROS2 适配点：参考文件把 0.2 写死；这里仍然读 reverse_check_distance_，但
  // nav2_params.yaml 默认会直接写成 0.20，从行为上等价于参考实现。
  const auto check_pose = getLookaheadPoint(plan, robot_pose, reverse_check_distance_, false);
  geometry_msgs::msg::PoseStamped check_in_base;
  if (!transformPose(check_pose, robot_base_frame_, check_in_base)) {
    return false;
  }
  return check_in_base.pose.position.x < 0.0;
}

double OrigincarAckermannPurePursuitController::steeringAngle(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const geometry_msgs::msg::PoseStamped & target,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  // 对应参考文件 calculateSteering(target, robot_pose) 的 TF 变换段。
  // 纯追踪几何必须在车体坐标系下计算：x 是车头前方，y 是车体左侧。
  // ROS2 适配点：参考文件用 tf_->transform(target, ..., robot_base_frame_, ros::Time(0), ...)
  // 这里通过 transformPose() 统一做 stamp=0 的最新 TF 查询，目标帧仍是 robot_base_frame_。
  geometry_msgs::msg::PoseStamped target_in_robot;
  if (!transformPose(target, robot_base_frame_, target_in_robot)) {
    throw nav2_core::PlannerException("Failed to transform pure pursuit target to robot frame");
  }

  // 对应参考文件中的:
  //   double y = target_in_robot.pose.position.y;
  //   double x = target_in_robot.pose.position.x;
  //   double L_sq = x*x + y*y;
  const double x = target_in_robot.pose.position.x;
  const double y = target_in_robot.pose.position.y;
  const double distance_sq = x * x + y * y;

  // 对应参考文件 MIN_DIST_SQ = 0.0001。目标点和车体几乎重合时直接不转向，
  // 避免除零，也避免目标点抖动导致方向盘在原地来回打。
  if (distance_sq < 1e-4) {
    return 0.0;
  }

  // 对应参考文件纯追踪公式：
  //   alpha = atan2(y, x)
  //   angular_z = atan(2.0 * CARL_ * sin(alpha) / sqrt(L_sq)) * 57.3
  //
  // 这里的 angular_z 变量名沿用参考文件语义：它实际表示“前轮等效转角，单位度”，
  // 并不是 ROS /cmd_vel.angular.z。真正的 cmd_vel.angular.z 在 computeVelocityCommands()
  // 末尾按阿克曼模型从这个度制转角换算出来。
  const double alpha = std::atan2(y, x);
  double angular_z =
    std::atan(2.0 * wheelbase_ * std::sin(alpha) / std::sqrt(distance_sq)) *
    kReferenceRadiansToDegrees;

  // 对应参考文件“接近终点时的位姿校正增强（仅在前进模式下）”。
  // 参考 ROS1 版本会重新从 costmap 取 robot_pose；ROS2 Nav2 已经把当前位姿作为入参
  // 传给 computeVelocityCommands()，所以这里直接使用同一个 robot_pose，避免二次查询。
  if (!reverse_mode_ && !plan.empty()) {
    const double dist_to_goal = poseDistance(plan.back(), robot_pose);
    if (dist_to_goal < kReferencePoseAdjustDistance) {
      const double goal_yaw = tf2::getYaw(plan.back().pose.orientation);
      const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
      const double angle_error = angles::shortest_angular_distance(robot_yaw, goal_yaw);

      // 参考文件常量 kp_orientation = 1.5，并把弧度误差乘 57.3 转成度制校正量。
      const double orientation_correction =
        kReferenceOrientationKp * angle_error * kReferenceRadiansToDegrees;

      // 参考文件使用 max(0.3, 1.0 - dist_to_goal / 0.25) 作为融合权重。
      // 旧 ROS2 版本用可配置参数和 clamp，这里恢复为参考常量和 std::max。
      const double correction_weight =
        std::max(0.3, 1.0 - dist_to_goal / kReferencePoseAdjustDistance);
      angular_z =
        angular_z * (1.0 - correction_weight) + orientation_correction * correction_weight;
    }
  }

  // 对应参考文件最后的角度限幅：
  //   return angular_z < -max_angular_ ? -max_angular_ :
  //          angular_z >  max_angular_ ?  max_angular_ : angular_z;
  //
  // max_angular_ 在参数中按参考文件语义使用“度”，不是现有旧版 max_steering_angle 的弧度。
  return (angular_z < -max_angular_) ? -max_angular_ :
         (angular_z > max_angular_) ? max_angular_ :
         angular_z;
}

double OrigincarAckermannPurePursuitController::targetLinearVelocity(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  double steering_angle_degrees) const
{
  // 对应参考文件 acalculateVel(target_pose, angular_z) 开头：
    // max_vel = is_reverse_mode_ ? max_reverse_velocity_ : max_linear_velocity_
    // current_vel = fabs(current_velocity_)
    // target_linear_vel = max_vel
  //
  // 这里的返回值始终是“速度幅值”，因此是非负数。前进/倒车的符号在
  // computeVelocityCommands() 输出 TwistStamped 时再按参考文件统一加上。
  const double max_velocity = reverse_mode_ ? max_reverse_velocity_ : max_linear_velocity_;
  const double current_velocity = std::fabs(current_velocity_);
  double target_velocity = max_velocity;

  // 对应参考文件“转弯减速功能”。steering_angle_degrees 已经是 calculateSteering()
  // 返回的度制 angular_z，因此 decel_angle_threshold_、max_angular_ 也必须按度理解。
  const double abs_angular = std::fabs(steering_angle_degrees);
  if (abs_angular > decel_angle_threshold_) {
    const double angle_ratio = std::min(abs_angular / max_angular_, 1.0);

    // 参考公式：角度占比越大，速度越小；平方项让减速更平滑。
    double turn_speed_factor = 1.0 - (angle_ratio * angle_ratio * decel_ratio_);
    turn_speed_factor = std::max(turn_speed_factor, kReferenceMinTurnSpeedFactor);

    target_velocity = max_velocity * turn_speed_factor;

    // 参考文件变量名叫 max_angular_vel_，注释中实际把它当作“转弯时线速度上限”使用。
    // 为了完整复刻，ROS2 参数也保留这个名字和语义。
    target_velocity = std::min(target_velocity, max_angular_vel_);

  }

  // 对应参考文件“接近终点时的线性减速逻辑（仅在前进模式下生效）”。
  // ROS2 适配点：goal_pose 来自 tracking_plan.back()，通常已经在 global_frame_；
  // 这里仍通过 transformPose() 转到 robot_pose 当前帧，保持多坐标系输入时的安全性。
  // if (!reverse_mode_) {
  //   geometry_msgs::msg::PoseStamped goal;
  //   if (transformPose(goal_pose, robot_pose.header.frame_id, goal)) {
  //     const double dist_to_goal = poseDistance(goal, robot_pose);

  //     if (dist_to_goal < kReferenceGoalDecelDistance) {
  //       // 参考常量：终点前 0.3m 开始线性减速，最低速度 0.05m/s。
  //       const double speed_ratio = dist_to_goal / kReferenceGoalDecelDistance;
  //       target_velocity =
  //         kReferenceMinApproachSpeed +
  //         (max_velocity - kReferenceMinApproachSpeed) * speed_ratio;

  //       // 参考逻辑：如果终点 yaw 偏差大于 0.2rad，则继续按角度误差降低速度，
  //       // 便于接近目标时慢下来做姿态校正。
  //       const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  //       const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  //       const double angle_diff =
  //         std::fabs(angles::shortest_angular_distance(robot_yaw, goal_yaw));

  //       if (angle_diff > kReferenceYawSlowdownThreshold) {
  //         const double angle_factor = std::max(
  //           0.3,
  //           1.0 - (angle_diff - kReferenceYawSlowdownThreshold) /
  //           (M_PI - kReferenceYawSlowdownThreshold));
  //         target_velocity *= angle_factor;
  //       }

  //       target_velocity = std::max(target_velocity, kReferenceMinApproachSpeed);
  //       return target_velocity;
  //     }
  //   }
  // }

  // 对应参考文件“速度差过大时限制加速度”。参考注释写的是限制减速，
  // 但实际条件是 target_linear_vel - current_vel > decel_velocity_threshold_，
  // 即目标速度比当前速度高太多时，只允许本周期最多增加该阈值。
  if ((target_velocity - current_velocity) > decel_velocity_threshold_) {
    target_velocity = current_velocity + decel_velocity_threshold_;
  }

  // 对应参考文件最后的最大速度限制。这里不额外做旧 ROS2 版本的 clamp 下限，
  // 只保持参考实现“不能超过 max_vel”的语义。
  if (target_velocity > max_velocity) {
    target_velocity = max_velocity;
  }

  return target_velocity;
}

void OrigincarAckermannPurePursuitController::publishDebug(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const geometry_msgs::msg::PoseStamped & target_pose,
  bool goal_reached)
{
  // 生命周期 publisher 只有 activate 后才能发布；这里逐个检查，避免状态切换时访问空指针。
  if (local_plan_pub_ && local_plan_pub_->is_activated()) {
    nav_msgs::msg::Path local_plan;
    local_plan.header.stamp = node_->now();
    local_plan.header.frame_id = plan.empty() ? target_pose.header.frame_id : plan.front().header.frame_id;
    local_plan.poses = plan;
    local_plan_pub_->publish(local_plan);
  }

  if (target_pose_pub_ && target_pose_pub_->is_activated()) {
    auto target = target_pose;
    target.header.stamp = node_->now();
    target_pose_pub_->publish(target);
  }

  if (goal_reached_pub_ && goal_reached_pub_->is_activated() && goal_reached != last_goal_reached_) {
    std_msgs::msg::Bool msg;
    msg.data = goal_reached;
    goal_reached_pub_->publish(msg);
    last_goal_reached_ = goal_reached;
  }
}

}  // namespace origincar_navigation

PLUGINLIB_EXPORT_CLASS(
  origincar_navigation::OrigincarAckermannPurePursuitController,
  nav2_core::Controller)
