#ifndef ORIGINCAR_NAVIGATION__ORIGINCAR_GOAL_CHECKER_HPP_
#define ORIGINCAR_NAVIGATION__ORIGINCAR_GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace origincar_navigation
{

// Nav2 官方 SimpleGoalChecker 默认带有 stateful 语义：一旦先满足 xy，
// 后续只继续检查 yaw。这与本项目控制器内部“位置和朝向同时满足才到点”
// 的判定不一致，因此需要一个和控制器完全同源的 GoalChecker。
class OrigincarGoalChecker : public nav2_core::GoalChecker
{
public:
  OrigincarGoalChecker() = default;
  ~OrigincarGoalChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  std::string scopedParam(const std::string & param_name) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  double goal_tolerance_{0.08};
  double yaw_goal_tolerance_{1.05};
};

}  // namespace origincar_navigation

#endif  // ORIGINCAR_NAVIGATION__ORIGINCAR_GOAL_CHECKER_HPP_
