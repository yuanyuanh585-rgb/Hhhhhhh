#include "origincar_navigation/origincar_goal_checker.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "origincar_navigation/goal_reached_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

}  // namespace

void OrigincarGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock lifecycle node in OrigincarGoalChecker");
  }

  plugin_name_ = plugin_name;
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("goal_tolerance"), rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node_, scopedParam("yaw_goal_tolerance"), rclcpp::ParameterValue(1.05));

  node_->get_parameter(scopedParam("goal_tolerance"), goal_tolerance_);
  node_->get_parameter(scopedParam("yaw_goal_tolerance"), yaw_goal_tolerance_);
}

void OrigincarGoalChecker::reset()
{
}

bool OrigincarGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  // 此处故意和控制器共享完全同源的终点判定逻辑，确保停车条件和 action
  // success 条件来自同一个真值来源。
  return origincar_navigation::isGoalReached(
    query_pose, goal_pose, goal_tolerance_, yaw_goal_tolerance_);
}

bool OrigincarGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  pose_tolerance = geometry_msgs::msg::Pose();
  pose_tolerance.position.x = goal_tolerance_;
  pose_tolerance.position.y = goal_tolerance_;
  pose_tolerance.orientation = quaternionFromYaw(yaw_goal_tolerance_);

  const double invalid = std::numeric_limits<double>::lowest();
  vel_tolerance = geometry_msgs::msg::Twist();
  vel_tolerance.linear.x = invalid;
  vel_tolerance.linear.y = invalid;
  vel_tolerance.linear.z = invalid;
  vel_tolerance.angular.x = invalid;
  vel_tolerance.angular.y = invalid;
  vel_tolerance.angular.z = invalid;
  return true;
}

std::string OrigincarGoalChecker::scopedParam(const std::string & param_name) const
{
  return plugin_name_ + "." + param_name;
}

}  // namespace origincar_navigation

PLUGINLIB_EXPORT_CLASS(
  origincar_navigation::OrigincarGoalChecker,
  nav2_core::GoalChecker)
