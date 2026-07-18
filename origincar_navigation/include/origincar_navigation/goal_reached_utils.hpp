#ifndef ORIGINCAR_NAVIGATION__GOAL_REACHED_UTILS_HPP_
#define ORIGINCAR_NAVIGATION__GOAL_REACHED_UTILS_HPP_

#include <cmath>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"

namespace origincar_navigation
{

// Nav2 action 成功条件和控制器停车条件必须使用同一套终点判定，
// 否则会出现“控制器已停住但 action 还未成功”的语义分裂。
inline bool isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  double goal_tolerance,
  double yaw_goal_tolerance)
{
  const double dx = goal_pose.position.x - query_pose.position.x;
  const double dy = goal_pose.position.y - query_pose.position.y;
  if (std::hypot(dx, dy) > goal_tolerance) {
    return false;
  }

  const double query_yaw = tf2::getYaw(query_pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  return std::fabs(angles::shortest_angular_distance(query_yaw, goal_yaw)) <=
         yaw_goal_tolerance;
}

}  // namespace origincar_navigation

#endif  // ORIGINCAR_NAVIGATION__GOAL_REACHED_UTILS_HPP_
