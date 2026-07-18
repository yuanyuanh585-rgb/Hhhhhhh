#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "origincar_navigation/tracking_route_provider.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace origincar_navigation
{
namespace
{

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

double normalizedYaw(double yaw)
{
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

}  // namespace

class OrigincarAckermannController : public nav2_core::Controller
{
public:
  OrigincarAckermannController() = default;
  ~OrigincarAckermannController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    // 控制器插件由 controller_server 生命周期节点创建。configure 只保存依赖、
    // 读取参数和创建调试 publisher，不在这里发布任何运动指令。
    node_ = parent.lock();
    if (!node_) {
      throw std::runtime_error("Failed to lock lifecycle node in OrigincarAckermannController");
    }

    name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    loadParameters();
    resetTrackingState();

    // 控制算法参数使用自己的控制器命名空间，路径提供器和局部避障参数使用共享
    // tracking_route_provider 命名空间，确保两个控制器看到同一套避障配置。
    tracking_route_provider_ = createTrackingRouteProvider(tracking_route_provider_plugin_);
    tracking_route_provider_->configure(
      node_, "tracking_route_provider", name_, tf_, costmap_ros_, transform_tolerance_);

    // local_plan 显示本周期实际跟踪的路径；target_pose 显示前视目标点；
    // goal_reached 只在内部终点状态变化时发布，便于调试停车条件。
    local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(name_ + "/local_plan", 1);
    target_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/target_pose", 1);
    goal_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(name_ + "/goal_reached", 1);

    RCLCPP_INFO(
      node_->get_logger(),
      "Configured %s distance-yaw controller: wheelbase=%.3f m, max_steering=%.3f deg, "
      "k_w_max=%.3f",
      name_.c_str(), wheelbase_, max_steering_angle_deg_, k_w_max_);
  }

  void cleanup() override
  {
    // cleanup 后插件可能再次 configure，因此清空 publisher、路径缓存和跨周期控制状态。
    local_plan_pub_.reset();
    target_pose_pub_.reset();
    goal_reached_pub_.reset();
    if (tracking_route_provider_) {
      tracking_route_provider_->cleanup();
    }
    base_plan_ = nav_msgs::msg::Path();
    resetTrackingState();
    RCLCPP_INFO(node_->get_logger(), "Cleaned up %s", name_.c_str());
  }

  void activate() override
  {
    local_plan_pub_->on_activate();
    target_pose_pub_->on_activate();
    goal_reached_pub_->on_activate();
    if (tracking_route_provider_) {
      tracking_route_provider_->activate();
    }
    RCLCPP_INFO(node_->get_logger(), "Activated %s", name_.c_str());
  }

  void deactivate() override
  {
    local_plan_pub_->on_deactivate();
    target_pose_pub_->on_deactivate();
    goal_reached_pub_->on_deactivate();
    if (tracking_route_provider_) {
      tracking_route_provider_->deactivate();
    }
    RCLCPP_INFO(node_->get_logger(), "Deactivated %s", name_.c_str());
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    // 每次收到新路径都从第一个路径点重新跟踪，并清空上一条路径留下的滤波速度和
    // 终点到达标志，防止旧路径的下标或停车状态影响新目标。
    base_plan_ = path;
    if (tracking_route_provider_) {
      tracking_route_provider_->resetBasePlan(path);
    }
    resetTrackingState();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = robot_base_frame_;

    if (base_plan_.poses.empty()) {
      throw nav2_core::PlannerException("OrigincarAckermannController received an empty plan");
    }

    geometry_msgs::msg::PoseStamped robot_pose = pose;
    if (robot_pose.header.frame_id.empty()) {
      robot_pose.header.frame_id = base_plan_.header.frame_id;
    }
    if (robot_pose.header.frame_id != global_frame_) {
      geometry_msgs::msg::PoseStamped transformed_robot_pose;
      if (!transformPose(robot_pose, global_frame_, transformed_robot_pose)) {
        throw nav2_core::PlannerException("Failed to transform robot pose to controller frame");
      }
      robot_pose = transformed_robot_pose;
    }

    const auto tracking_path = tracking_route_provider_ ?
      tracking_route_provider_->getTrackingPath(robot_pose) : base_plan_;
    if (tracking_path.poses.empty()) {
      throw nav2_core::PlannerException("Tracking route provider returned an empty path");
    }

    const auto plan = transformedPlan(tracking_path, global_frame_);
    if (plan.empty()) {
      throw nav2_core::PlannerException("Failed to transform tracking path");
    }

    // 当前速度只用于动态前视距离。倒车时里程计速度可能是负数，后续使用绝对值，
    // 使前进和倒车在同等速度大小下有一致的前视长度。
    current_velocity_ = velocity.linear.x;
    reverse_mode_ = shouldReverse(plan, robot_pose);

    const double lookahead_distance = adaptiveLookaheadDistance(current_velocity_);
    const auto target_pose = getLookaheadPoint(plan, robot_pose, lookahead_distance, true);

    const ControlOutput control = computePathTrackingControl(plan, target_pose, robot_pose);
    cmd_vel.twist.linear.x = reverse_mode_ ? -control.linear_velocity : control.linear_velocity;
    cmd_vel.twist.angular.z = control.angular_velocity;

    current_velocity_ = cmd_vel.twist.linear.x;
    publishDebug(plan, target_pose, control.goal_reached);
    return cmd_vel;
  }

  void setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) override
  {
    // 本控制器使用自己的独立速度参数，不接入 Nav2 speed filter 的动态限速接口。
  }

private:
  struct ControlOutput
  {
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    bool goal_reached{false};
  };

  std::string scopedParam(const std::string & param_name) const
  {
    return name_ + "." + param_name;
  }

  void loadParameters()
  {
    // 这些参数全部挂在当前控制器 ID 名下。默认值就是新算法的完整参数集合，
    // 即使旧控制器的 PurePursuit 参数存在，也不会被这里读取。
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("d_min"), rclcpp::ParameterValue(0.50));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("k_gain"), rclcpp::ParameterValue(1.00));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("d_max"), rclcpp::ParameterValue(3.00));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("v_max"), rclcpp::ParameterValue(1.00));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("yaw_gain"), rclcpp::ParameterValue(1.50));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("align_distance"), rclcpp::ParameterValue(0.60));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("terminal_speed_scale"), rclcpp::ParameterValue(0.60));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("dist_tolerance"), rclcpp::ParameterValue(0.10));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("yaw_tolerance"), rclcpp::ParameterValue(0.02));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("deadband"), rclcpp::ParameterValue(0.02));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("velocity_filter_old_weight"), rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("wheelbase"), rclcpp::ParameterValue(0.143));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("max_steering_angle_deg"), rclcpp::ParameterValue(18.0));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("reverse_check_distance"), rclcpp::ParameterValue(0.20));
    nav2_util::declare_parameter_if_not_declared(
      node_, scopedParam("transform_tolerance"), rclcpp::ParameterValue(0.20));
    nav2_util::declare_parameter_if_not_declared(
      node_, "tracking_route_provider.plugin",
      rclcpp::ParameterValue("origincar_navigation/SegmentBypassTrackingRouteProvider"));

    node_->get_parameter(scopedParam("d_min"), d_min_);
    node_->get_parameter(scopedParam("k_gain"), k_gain_);
    node_->get_parameter(scopedParam("d_max"), d_max_);
    node_->get_parameter(scopedParam("v_max"), v_max_);
    node_->get_parameter(scopedParam("yaw_gain"), yaw_gain_);
    node_->get_parameter(scopedParam("align_distance"), align_distance_);
    node_->get_parameter(scopedParam("terminal_speed_scale"), terminal_speed_scale_);
    node_->get_parameter(scopedParam("dist_tolerance"), dist_tolerance_);
    node_->get_parameter(scopedParam("yaw_tolerance"), yaw_tolerance_);
    node_->get_parameter(scopedParam("deadband"), deadband_);
    node_->get_parameter(scopedParam("velocity_filter_old_weight"), velocity_filter_old_weight_);
    node_->get_parameter(scopedParam("wheelbase"), wheelbase_);
    node_->get_parameter(scopedParam("max_steering_angle_deg"), max_steering_angle_deg_);
    node_->get_parameter(scopedParam("reverse_check_distance"), reverse_check_distance_);
    node_->get_parameter(scopedParam("transform_tolerance"), transform_tolerance_);
    node_->get_parameter("tracking_route_provider.plugin", tracking_route_provider_plugin_);

    d_min_ = std::max(d_min_, 0.0);
    k_gain_ = std::max(k_gain_, 0.0);
    d_max_ = std::max(d_max_, d_min_);
    v_max_ = std::max(v_max_, 0.0);
    align_distance_ = std::max(align_distance_, 1e-3);
    terminal_speed_scale_ = std::max(terminal_speed_scale_, 0.0);
    dist_tolerance_ = std::max(dist_tolerance_, 0.0);
    yaw_tolerance_ = std::max(yaw_tolerance_, 0.0);
    deadband_ = std::max(deadband_, 0.0);
    velocity_filter_old_weight_ = std::clamp(velocity_filter_old_weight_, 0.0, 1.0);
    wheelbase_ = std::max(wheelbase_, 1e-3);
    max_steering_angle_deg_ = std::max(max_steering_angle_deg_, 0.0);
    reverse_check_distance_ = std::max(reverse_check_distance_, 0.0);

    // 角速度限幅系数表示“单位线速度下允许的最大 yaw rate”。实车约束来自最大前轮
    // 转角，因此按自行车模型 omega/v = tan(delta) / wheelbase 换算。
    const double max_steering_angle_rad = max_steering_angle_deg_ * M_PI / 180.0;
    k_w_max_ = std::tan(max_steering_angle_rad) / wheelbase_;
  }

  void resetTrackingState()
  {
    track_index_ = 0;
    position_reached_ = false;
    filtered_velocity_ = 0.0;
    reverse_mode_ = false;
    current_velocity_ = 0.0;
    last_goal_reached_ = false;
  }

  bool transformPose(
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
      // 路径点通常来自较早的规划时刻。stamp 置零表示使用最新可用 TF，
      // 避免静态路径在运行一段时间后因为旧时间戳查询失败。
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

  std::vector<geometry_msgs::msg::PoseStamped> transformedPlan(
    const nav_msgs::msg::Path & path,
    const std::string & target_frame) const
  {
    // 返回转换后的副本，路径提供器内部维护的 active route 不会被控制器改写。
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

  size_t closestPoseIndex(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & pose) const
  {
    // 用欧氏距离查找当前最近路径点。后续前视点从最近点往路径末端搜索，
    // 可以避免机器人已经走过的点反复成为目标。
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < plan.size(); ++i) {
      const double dist = poseDistance(plan[i], pose);
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }
    return closest_idx;
  }

  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lookahead_distance,
    bool allow_goal_extension) const
  {
    // const size_t closest_idx = closestPoseIndex(plan, robot_pose);
    const size_t closest_idx = 0;

    // 从最近路径点开始查找第一个离机器人至少 lookahead_distance 的点。
    // 这里比较的是机器人到候选点的直线距离，不累加路径弧长。
    for (size_t i = closest_idx; i < plan.size(); ++i) {
      if (poseDistance(plan[i], robot_pose) >= lookahead_distance) {
        return plan[i];
      }
    }

    if (!allow_goal_extension) {
      // 倒车判定只需要真实路径上的短前视点。路径剩余过短时直接使用终点，
      // 避免虚拟延长点改变“目标在车前还是车后”的判断。
      return plan.back();
    }

    // 路径末端不足前视距离时，沿终点朝向延长一个虚拟点。该点只参与本周期控制，
    // 不写回路径，从而让终点附近仍然有稳定的目标方向。
    auto virtual_point = plan.back();
    double goal_yaw = tf2::getYaw(virtual_point.pose.orientation);
    if (reverse_mode_) {
      goal_yaw = normalizedYaw(goal_yaw + M_PI);
    }

    const double dist_to_goal = poseDistance(virtual_point, robot_pose);
    const double extend_distance = lookahead_distance - dist_to_goal + 0.10;
    if (extend_distance > 0.0) {
      virtual_point.pose.position.x += extend_distance * std::cos(goal_yaw);
      virtual_point.pose.position.y += extend_distance * std::sin(goal_yaw);
      virtual_point.pose.orientation = quaternionFromYaw(goal_yaw);
    }

    return virtual_point;
  }

  double adaptiveLookaheadDistance(double current_speed) const
  {
    // 速度越大，前视点越远；最低前视距离保证低速时也不是盯着车头附近的点，
    // 最高前视距离避免高速或异常速度下直接跳到过远路径点。
    return std::min(d_max_, d_min_ + k_gain_ * std::fabs(current_speed));
  }

  bool shouldReverse(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose) const
  {
    // 用很短的前视点判断本周期目标落在车体前方还是后方。转换到车体坐标系后，
    // x < 0 表示目标在车尾方向，控制器应倒车跟踪而不是掉头寻找目标。
    const auto check_pose = getLookaheadPoint(plan, robot_pose, reverse_check_distance_, false);
    geometry_msgs::msg::PoseStamped check_in_base;
    if (!transformPose(check_pose, robot_base_frame_, check_in_base)) {
      return false;
    }
    return check_in_base.pose.position.x < 0.0;
  }

  ControlOutput computePathTrackingControl(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    ControlOutput output;

    // track_index_ 保存跨周期路径进度。收到新路径时会归零；正常跟踪时只向后推进，
    // 这样车辆不会因为定位噪声反复追已经越过的路径点。
    if (track_index_ >= plan.size()) {
      track_index_ = plan.size() - 1;
    }

    const double lookahead_distance = adaptiveLookaheadDistance(current_velocity_);
    while (track_index_ < plan.size() &&
      poseDistance(robot_pose, plan[track_index_]) < lookahead_distance)
    {
      ++track_index_;
    }
    if (track_index_ == plan.size()) {
      --track_index_;
    }

    const bool tracking_last_pose = track_index_ == plan.size() - 1;
    const geometry_msgs::msg::PoseStamped & tracking_pose =
      tracking_last_pose ? plan.back() : plan[track_index_];

    // 位置误差全部在全局坐标系下计算。倒车时使用车尾朝向作为当前跟踪方向，
    // 因而车后方路径点会形成小角度误差，不会迫使车辆原地转向。
    const double robot_x = robot_pose.pose.position.x;
    const double robot_y = robot_pose.pose.position.y;
    const double tracking_x = tracking_pose.pose.position.x;
    const double tracking_y = tracking_pose.pose.position.y;
    const double dx = tracking_x - robot_x;
    const double dy = tracking_y - robot_y;
    const double dist = std::hypot(dx, dy);
    const double yaw_des = std::atan2(dy, dx);
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double control_yaw = reverse_mode_ ? normalizedYaw(robot_yaw + M_PI) : robot_yaw;

    double yaw_path_err = angles::shortest_angular_distance(control_yaw, yaw_des);
    double yaw_err = yaw_path_err;
    double v_cmd = 0.0;
    double w_cmd = 0.0;

    if (tracking_last_pose) {
      // 终点附近不只追坐标，还要逐渐对齐路径最后一个点的朝向。alpha=1 时主要朝向
      // 终点坐标，alpha=0 时完全按终点姿态控制，使停车前姿态过渡更平滑。
      const double goal_yaw = tf2::getYaw(plan.back().pose.orientation);
      const double goal_control_yaw = reverse_mode_ ? normalizedYaw(goal_yaw + M_PI) : goal_yaw;
      const double yaw_goal_err =
        angles::shortest_angular_distance(control_yaw, goal_control_yaw);

      const double alpha = std::clamp(dist / align_distance_, 0.0, 1.0);
      yaw_err = alpha * yaw_path_err + (1.0 - alpha) * yaw_goal_err;
      if (dist <= align_distance_ / 2.0) {
        yaw_err = yaw_goal_err;
      }

      v_cmd = terminal_speed_scale_ * alpha;
      w_cmd = yaw_gain_ * yaw_err;

      // 进入距离容差后先记住“位置到达”。后续即使定位抖动让 dist 轻微变大，
      // 也允许直接结束，避免在终点附近反复补偿。
      if (dist <= dist_tolerance_) {
        position_reached_ = true;
      }

      // 保持原有完成策略：位置到达后，只要姿态误差满足容差，或者距离减速已经把
      // 目标速度压到很小，就立即停车并报告完成。
      if (position_reached_ && (yaw_err <= yaw_tolerance_ || v_cmd < deadband_)) {
        filtered_velocity_ = 0.0;
        output.goal_reached = true;
        return output;
      }

      if (position_reached_ && dist > dist_tolerance_) {
        filtered_velocity_ = 0.0;
        output.goal_reached = true;
        return output;
      }
    } else {
      // 中间路径段按航向误差降速：45 度以内线性降低，超过 45 度按最大降速处理。
      // 速度最低为 0.5 * v_max，保证大角度追踪时不会高速前冲。
      const double alpha = std::clamp(std::fabs(yaw_err) / (M_PI / 4.0), 0.0, 1.0);
      v_cmd = v_max_ * (1.0 - alpha / 2.0);
      w_cmd = yaw_gain_ * yaw_err;
    }

    // 角速度上限随线速度成比例变化。线速度越低，允许的 yaw rate 越小，
    // 这能抑制低速终点阶段过大的方向指令。
    const double w_limit = k_w_max_ * std::fabs(v_cmd);
    w_cmd = std::clamp(w_cmd, -w_limit, w_limit);

    if (std::fabs(v_cmd) < deadband_) {
      v_cmd = 0.0;
    }
    if (std::fabs(w_cmd) < deadband_) {
      w_cmd = 0.0;
    }

    // 一阶滤波只作用于线速度，方向控制保持本周期计算值。目标速度为零时直接清空
    // 历史速度，确保停车干脆，不被滤波尾巴拖住。
    filtered_velocity_ =
      velocity_filter_old_weight_ * filtered_velocity_ +
      (1.0 - velocity_filter_old_weight_) * v_cmd;
    if (std::fabs(filtered_velocity_) < deadband_ || v_cmd == 0.0) {
      filtered_velocity_ = 0.0;
    }

    output.linear_velocity = filtered_velocity_;
    output.angular_velocity = w_cmd;

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[%s] idx=%zu/%zu reverse=%s target_dist=%.3f target_x=%.3f target_y=%.3f "
      "lookahead_x=%.3f lookahead_y=%.3f v=%.3f w=%.3f",
      name_.c_str(), track_index_, plan.size(), reverse_mode_ ? "true" : "false",
      dist, tracking_x, tracking_y, target_pose.pose.position.x, target_pose.pose.position.y,
      output.linear_velocity, output.angular_velocity);

    return output;
  }

  void publishDebug(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & target_pose,
    bool goal_reached)
  {
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

    if (goal_reached_pub_ && goal_reached_pub_->is_activated() &&
      goal_reached != last_goal_reached_)
    {
      std_msgs::msg::Bool msg;
      msg.data = goal_reached;
      goal_reached_pub_->publish(msg);
      last_goal_reached_ = goal_reached;
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  std::unique_ptr<TrackingRouteProvider> tracking_route_provider_;

  std::string name_;
  std::string robot_base_frame_{"base_footprint"};
  std::string global_frame_{"map"};
  std::string tracking_route_provider_plugin_{
    "origincar_navigation/SegmentBypassTrackingRouteProvider"};
  nav_msgs::msg::Path base_plan_;

  double d_min_{0.50};
  double k_gain_{1.00};
  double d_max_{3.00};
  double v_max_{1.00};
  double yaw_gain_{1.50};
  double align_distance_{0.60};
  double terminal_speed_scale_{0.60};
  double dist_tolerance_{0.10};
  double yaw_tolerance_{0.02};
  double deadband_{0.02};
  double velocity_filter_old_weight_{0.25};
  double wheelbase_{0.143};
  double max_steering_angle_deg_{18.0};
  double k_w_max_{2.272};
  double reverse_check_distance_{0.20};
  double transform_tolerance_{0.20};

  double current_velocity_{0.0};
  double filtered_velocity_{0.0};
  size_t track_index_{0};
  bool position_reached_{false};
  bool reverse_mode_{false};
  bool last_goal_reached_{false};
};

}  // namespace origincar_navigation

PLUGINLIB_EXPORT_CLASS(
  origincar_navigation::OrigincarAckermannController,
  nav2_core::Controller)
