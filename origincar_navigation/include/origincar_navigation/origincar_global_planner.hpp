#ifndef ORIGINCAR_NAVIGATION__ORIGINCAR_GLOBAL_PLANNER_HPP_
#define ORIGINCAR_NAVIGATION__ORIGINCAR_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "yaml-cpp/yaml.h"

namespace origincar_navigation
{

class OrigincarGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  OrigincarGlobalPlanner() = default;
  ~OrigincarGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  using RouteMap = std::unordered_map<int, std::vector<geometry_msgs::msg::PoseStamped>>;

  void loadParameters();
  bool loadRoutes();
  bool parseRoute(int route_id, const std::string & route_name, const YAML::Node & config);
  size_t firstConfiguredPointIndex(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<geometry_msgs::msg::PoseStamped> & route_points,
    int route_id) const;
  geometry_msgs::msg::PoseStamped parseRoutePoint(
    const YAML::Node & point_node,
    const std::string & route_name,
    size_t point_index) const;
  nav_msgs::msg::Path generateSmoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints) const;
  void appendSmoothSegment(
    const geometry_msgs::msg::PoseStamped & prev,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    const geometry_msgs::msg::PoseStamped & next,
    std::vector<geometry_msgs::msg::PoseStamped> & poses) const;
  nav_msgs::msg::Path trimPathFromStart(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start) const;
  nav_msgs::msg::Path createDynamicReversePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  nav_msgs::msg::Path createForwardTransitionPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double target_yaw) const;
  nav_msgs::msg::Path createReverseTurnCandidate(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double prep_distance,
    double turn_radius) const;
  void appendReverseStraight(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    double distance) const;
  geometry_msgs::msg::PoseStamped appendReverseArc(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    double target_yaw,
    double turn_radius) const;
  void appendPathSkippingFirst(
    nav_msgs::msg::Path & path,
    const nav_msgs::msg::Path & suffix) const;
  void markDynamicTransitionPath(nav_msgs::msg::Path & path) const;
  bool isPathCollisionFree(const nav_msgs::msg::Path & path) const;
  nav_msgs::msg::Path createFallbackPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
  bool isDynamicReverseGoal(const geometry_msgs::msg::PoseStamped & goal) const;
  int routeIdFromGoal(const geometry_msgs::msg::PoseStamped & goal) const;
  std::string scopedParam(const std::string & param_name) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_navfn_planner::NavfnPlanner> fallback_planner_;

  std::string name_;
  std::string global_frame_;
  RouteMap routes_;

  double tension_{1.5};
  int min_interpolation_steps_{10};
  double path_resolution_factor_{10.0};
  double trim_distance_{0.15};
  double route_start_skip_distance_{0.3};
  double transition_sample_interval_{0.03};
  std::vector<double> transition_turn_radii_{0.40, 0.55};
  std::vector<double> transition_reverse_prep_distances_{0.0, 0.40, 0.70};
  std::string fallback_navfn_plugin_{"nav2_navfn_planner/NavfnPlanner"};
  std::string fallback_planner_name_;
  std::string routes_file_;
};

}  // namespace origincar_navigation

#endif  // ORIGINCAR_NAVIGATION__ORIGINCAR_GLOBAL_PLANNER_HPP_
