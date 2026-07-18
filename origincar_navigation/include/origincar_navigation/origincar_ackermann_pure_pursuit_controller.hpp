#ifndef ORIGINCAR_NAVIGATION__ORIGINCAR_ACKERMANN_PURE_PURSUIT_CONTROLLER_HPP_
#define ORIGINCAR_NAVIGATION__ORIGINCAR_ACKERMANN_PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "origincar_navigation/tracking_route_provider.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

namespace origincar_navigation
{

class OrigincarAckermannPurePursuitController : public nav2_core::Controller
{
public:
  OrigincarAckermannPurePursuitController() = default;
  ~OrigincarAckermannPurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker *) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  std::string scopedParam(const std::string & param_name) const;
  void loadParameters();

  bool transformPose(
    const geometry_msgs::msg::PoseStamped & in_pose,
    const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  std::vector<geometry_msgs::msg::PoseStamped> transformedPlan(
    const nav_msgs::msg::Path & path,
    const std::string & target_frame) const;

  size_t closestPoseIndex(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & pose) const;

  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lookahead_distance,
    bool allow_goal_extension) const;

  double adaptiveLookaheadDistance(double current_speed) const;
  bool shouldReverse(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  double steeringAngle(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & target,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  double targetLinearVelocity(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double steering_angle_degrees) const;
  void publishDebug(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const geometry_msgs::msg::PoseStamped & target_pose,
    bool goal_reached);

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
  std::string tracking_route_provider_plugin_{"origincar_navigation/SegmentBypassTrackingRouteProvider"};
  nav_msgs::msg::Path base_plan_;

  double min_lookahead_distance_{0.30};
  double max_lookahead_distance_{2.00};
  double lookahead_ratio_{0.50};
  double max_linear_velocity_{0.50};
  double max_reverse_velocity_{0.30};
  double reverse_lookahead_ratio_{1.00};
  double wheelbase_{0.143};
  double max_angular_{37.24};
  double goal_tolerance_{0.10};
  double decel_angle_threshold_{30.0};
  double decel_velocity_threshold_{0.20};
  double decel_ratio_{0.80};
  double max_angular_vel_{0.30};
  double reverse_check_distance_{0.20};
  double transform_tolerance_{0.20};

  double current_velocity_{0.0};
  bool reverse_mode_{false};
  bool last_goal_reached_{false};
};

}  // namespace origincar_navigation

#endif  // ORIGINCAR_NAVIGATION__ORIGINCAR_ACKERMANN_PURE_PURSUIT_CONTROLLER_HPP_
