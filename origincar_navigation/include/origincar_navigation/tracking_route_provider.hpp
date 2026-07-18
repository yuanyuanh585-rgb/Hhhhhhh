#ifndef ORIGINCAR_NAVIGATION__TRACKING_ROUTE_PROVIDER_HPP_
#define ORIGINCAR_NAVIGATION__TRACKING_ROUTE_PROVIDER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace origincar_navigation
{

class TrackingRouteProvider
{
public:
  virtual ~TrackingRouteProvider() = default;

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & param_namespace,
    const std::string & topic_prefix,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    double transform_tolerance) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual void resetBasePlan(const nav_msgs::msg::Path & path) = 0;
  virtual nav_msgs::msg::Path getTrackingPath(
    const geometry_msgs::msg::PoseStamped & robot_pose) = 0;
};

std::unique_ptr<TrackingRouteProvider> createTrackingRouteProvider(
  const std::string & plugin_name);

}  // namespace origincar_navigation

#endif  // ORIGINCAR_NAVIGATION__TRACKING_ROUTE_PROVIDER_HPP_
