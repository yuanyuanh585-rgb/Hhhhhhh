#ifndef ORIGINCAR_NAVIGATION__NOOP_PROGRESS_CHECKER_HPP_
#define ORIGINCAR_NAVIGATION__NOOP_PROGRESS_CHECKER_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace origincar_navigation
{

class NoopProgressChecker : public nav2_core::ProgressChecker
{
public:
  NoopProgressChecker() = default;
  ~NoopProgressChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

  void reset() override;
};

}  // namespace origincar_navigation

#endif  // ORIGINCAR_NAVIGATION__NOOP_PROGRESS_CHECKER_HPP_
