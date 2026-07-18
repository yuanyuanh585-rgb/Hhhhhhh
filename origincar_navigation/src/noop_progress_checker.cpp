#include "origincar_navigation/noop_progress_checker.hpp"

#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace origincar_navigation
{

void NoopProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
  const std::string & /*plugin_name*/)
{
}

bool NoopProgressChecker::check(geometry_msgs::msg::PoseStamped & /*current_pose*/)
{
  return true;
}

void NoopProgressChecker::reset()
{
}

}  // namespace origincar_navigation

PLUGINLIB_EXPORT_CLASS(
  origincar_navigation::NoopProgressChecker,
  nav2_core::ProgressChecker)
