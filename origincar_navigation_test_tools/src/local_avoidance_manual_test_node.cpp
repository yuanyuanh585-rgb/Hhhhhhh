#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "origincar_navigation/local_avoidance/segment_bypass_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

double distance2d(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}

}  // namespace

class LocalAvoidanceManualTestNode : public rclcpp::Node
{
public:
  LocalAvoidanceManualTestNode()
  : Node("local_avoidance_manual_test"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    loadParameters();
    planner_ = std::make_unique<origincar_navigation::local_avoidance::SegmentBypassPlanner>(
      options_);
    resetSimulatedPose();

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "input_global_path", 1,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg->poses.empty()) {
          global_path_ = *msg;
          global_path_.header.frame_id = normalizeFrame(global_path_.header.frame_id);
          RCLCPP_INFO(get_logger(), "Received input global path with %zu poses", msg->poses.size());
          updateOnce();
        }
      });

    nav_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 1,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (use_nav2_plan_ && !msg->poses.empty()) {
          global_path_ = *msg;
          global_path_.header.frame_id = normalizeFrame(global_path_.header.frame_id);
          updateOnce();
        }
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/goal_pose", 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        setGoalPose(*msg);
      });

    rviz_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        setGoalPose(*msg);
      });

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        setStartPose(pose);
      });

    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        geometry_msgs::msg::PointStamped point = *msg;
        point.header.frame_id = normalizeFrame(point.header.frame_id);
        if (point.header.frame_id != frame_id_) {
          try {
            point = tf_buffer_.transform(point, frame_id_, tf2::durationFromSec(0.2));
          } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(get_logger(), "Failed to transform clicked point: %s", ex.what());
            return;
          }
        }
        manual_obstacles_.push_back(point.point);
        RCLCPP_INFO(
          get_logger(), "Added obstacle #%zu at %.2f, %.2f",
          manual_obstacles_.size(), point.point.x, point.point.y);
        updateOnce();
      });

    clear_sub_ = create_subscription<std_msgs::msg::Empty>(
      "~/clear_obstacles", 1,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        manual_obstacles_.clear();
        RCLCPP_INFO(get_logger(), "Cleared manual obstacles");
        updateOnce();
      });

    play_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/play", 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        playback_enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "Offline playback %s", playback_enabled_ ? "started" : "paused");
      });

    pause_sub_ = create_subscription<std_msgs::msg::Empty>(
      "~/pause", 1,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        playback_enabled_ = false;
        RCLCPP_INFO(get_logger(), "Offline playback paused");
      });

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "~/start", 1,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        playback_enabled_ = true;
        RCLCPP_INFO(get_logger(), "Offline playback started");
      });

    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
      "~/reset_start", 1,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        playback_enabled_ = false;
        resetSimulatedPose();
        buildStraightGlobalPath();
        updateOnce();
        RCLCPP_INFO(get_logger(), "Offline start pose reset");
      });

    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/reference_path", 1);
    astar_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/astar_path", 1);
    optimized_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/optimized_path", 1);
    tracking_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/tracking_path", 1);
    obstacle_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/manual_obstacle_markers", 1);
    costmap_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("~/test_costmap_obstacles", 1);
    start_goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/start_goal_markers", 1);
    test_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/test_map", rclcpp::QoS(1).transient_local().reliable());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_frequency_, 0.1)),
      [this]() {
        advanceSimulatedPose();
        updateOnce();
      });

    buildStraightGlobalPath();
    RCLCPP_INFO(
      get_logger(),
      "Manual avoidance test is ready. Use RViz Publish Point on /clicked_point to add obstacles.");
  }

private:
  void loadParameters()
  {
    frame_id_ = declare_parameter("frame_id", "map");
    robot_frame_ = declare_parameter("robot_frame", "base_footprint");
    frame_id_ = normalizeFrame(frame_id_);
    robot_frame_ = normalizeFrame(robot_frame_);
    use_tf_pose_ = declare_parameter("use_tf_pose", true);
    use_nav2_plan_ = declare_parameter("use_nav2_plan", true);
    map_width_ = declare_parameter("map_width", 6.0);
    map_height_ = declare_parameter("map_height", 6.0);
    map_resolution_ = declare_parameter("map_resolution", 0.05);
    obstacle_radius_ = declare_parameter("obstacle_radius", 0.12);
    obstacle_inflation_radius_ = declare_parameter("obstacle_inflation_radius", 0.18);
    publish_frequency_ = declare_parameter("publish_frequency", 5.0);
    current_speed_ = declare_parameter("current_speed", 0.30);
    default_start_x_ = declare_parameter("default_start_x", -2.0);
    default_start_y_ = declare_parameter("default_start_y", -2.0);
    default_start_yaw_ = declare_parameter("default_start_yaw", 0.0);
    default_goal_x_ = declare_parameter("default_goal_x", 2.0);
    default_goal_y_ = declare_parameter("default_goal_y", 2.0);
    default_goal_yaw_ = declare_parameter("default_goal_yaw", 0.0);
    straight_path_resolution_ = declare_parameter("straight_path_resolution", 0.05);
    playback_enabled_ = declare_parameter("play_on_start", false);
    simulation_speed_ = declare_parameter("simulation_speed", 0.20);
    stable_path_reuse_enabled_ = declare_parameter("stable_path_reuse_enabled", true);
    stable_path_max_robot_distance_ = declare_parameter("stable_path_max_robot_distance", 0.35);
    stable_path_min_remaining_length_ = declare_parameter("stable_path_min_remaining_length", 0.40);

    options_.enabled = true;
    options_.replan_frequency = publish_frequency_;
    options_.scan_distance = declare_parameter("scan_distance", 2.0);
    options_.pre_margin = declare_parameter("pre_margin", 0.30);
    options_.post_margin = declare_parameter("post_margin", 0.50);
    options_.pre_point_margin = declare_parameter("pre_point_margin", 3);
    options_.post_point_margin = declare_parameter("post_point_margin", 5);
    options_.min_blocked_length = declare_parameter("min_blocked_length", 0.05);
    options_.min_replace_length = declare_parameter("min_replace_length", 0.40);
    options_.prune_search_distance = declare_parameter("prune_search_distance", 1.0);
    options_.smoothing_enabled = declare_parameter("bspline_enabled", true);
    options_.smoothing_collision_check =
      declare_parameter("bspline_collision_check_enabled", false);
    options_.astar_allow_unknown = declare_parameter("astar_allow_unknown", true);
    options_.astar_lethal_cost =
      static_cast<unsigned char>(declare_parameter("astar_lethal_cost", 253));
    options_.astar_cost_weight = declare_parameter("astar_cost_weight", 0.02);
    options_.astar_grid_downsample_factor = declare_parameter("astar_grid_downsample_factor", 1);
    options_.hybrid_wheelbase = declare_parameter("hybrid_wheelbase", 0.189);
    options_.hybrid_max_steering_angle = declare_parameter("hybrid_max_steering_angle", 0.65);
    options_.hybrid_yaw_bins = declare_parameter("hybrid_yaw_bins", 72);
    options_.hybrid_steering_samples = declare_parameter("hybrid_steering_samples", 5);
    options_.hybrid_step_distance = declare_parameter("hybrid_step_distance", 0.12);
    options_.hybrid_goal_tolerance = declare_parameter("hybrid_goal_tolerance", 0.12);
    options_.hybrid_yaw_tolerance = declare_parameter("hybrid_yaw_tolerance", 0.70);
    options_.hybrid_max_iterations = declare_parameter("hybrid_max_iterations", 20000);
    options_.hybrid_steering_cost_weight = declare_parameter("hybrid_steering_cost_weight", 0.05);
    options_.hybrid_goal_yaw_weight = declare_parameter("hybrid_goal_yaw_weight", 0.05);
    options_.hybrid_max_path_length_ratio = declare_parameter("hybrid_max_path_length_ratio", 3.0);
    options_.hybrid_search_lateral_margin =
      declare_parameter("hybrid_search_lateral_margin", 0.80);
    options_.sample_interval = declare_parameter("bspline_sample_interval", 0.05);
    options_.route_sample_interval = declare_parameter("route_sample_interval", 0.05);
  }

  std::string normalizeFrame(const std::string & frame) const
  {
    if (!frame.empty() && frame[0] == '/') {
      return frame.substr(1);
    }
    return frame;
  }

  geometry_msgs::msg::PoseStamped robotPose()
  {
    if (!use_tf_pose_) {
      auto pose = simulated_pose_;
      pose.header.stamp = now();
      return pose;
    }

    geometry_msgs::msg::PoseStamped pose = simulated_pose_;
    pose.header.stamp = now();

    try {
      const auto transform = tf_buffer_.lookupTransform(
        frame_id_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException &) {
      pose.header.stamp = now();
    }
    return pose;
  }

  void resetSimulatedPose()
  {
    simulated_pose_.header.frame_id = frame_id_;
    simulated_pose_.header.stamp = now();
    simulated_pose_.pose.position.x = default_start_x_;
    simulated_pose_.pose.position.y = default_start_y_;
    simulated_pose_.pose.orientation = yawToQuaternion(default_start_yaw_);
    last_update_time_ = now();
  }

  void setStartPose(geometry_msgs::msg::PoseStamped pose)
  {
    pose.header.frame_id = normalizeFrame(pose.header.frame_id);
    if (pose.header.frame_id != frame_id_) {
      try {
        pose = tf_buffer_.transform(pose, frame_id_, tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Failed to transform start pose: %s", ex.what());
        return;
      }
    }

    playback_enabled_ = false;
    stable_path_valid_ = false;
    stable_tracking_path_ = nav_msgs::msg::Path();
    simulated_pose_ = pose;
    simulated_pose_.header.frame_id = frame_id_;
    simulated_pose_.header.stamp = now();
    default_start_x_ = simulated_pose_.pose.position.x;
    default_start_y_ = simulated_pose_.pose.position.y;
    default_start_yaw_ = tf2::getYaw(simulated_pose_.pose.orientation);
    buildStraightGlobalPath();
    updateOnce();
    RCLCPP_INFO(
      get_logger(), "Set offline start pose to %.2f, %.2f",
      default_start_x_, default_start_y_);
  }

  void setGoalPose(geometry_msgs::msg::PoseStamped pose)
  {
    pose.header.frame_id = normalizeFrame(pose.header.frame_id);
    if (pose.header.frame_id != frame_id_) {
      try {
        pose = tf_buffer_.transform(pose, frame_id_, tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Failed to transform goal pose: %s", ex.what());
        return;
      }
    }

    goal_pose_ = pose;
    stable_path_valid_ = false;
    stable_tracking_path_ = nav_msgs::msg::Path();
    goal_pose_.header.frame_id = frame_id_;
    goal_pose_.header.stamp = now();
    default_goal_x_ = goal_pose_.pose.position.x;
    default_goal_y_ = goal_pose_.pose.position.y;
    default_goal_yaw_ = tf2::getYaw(goal_pose_.pose.orientation);
    buildStraightGlobalPath();
    updateOnce();
    RCLCPP_INFO(
      get_logger(), "Set offline local target to %.2f, %.2f",
      default_goal_x_, default_goal_y_);
  }

  void advanceSimulatedPose()
  {
    const auto current_time = now();
    const double dt = std::clamp((current_time - last_update_time_).seconds(), 0.0, 0.5);
    last_update_time_ = current_time;
    if (use_tf_pose_ || !playback_enabled_ || last_tracking_path_.poses.size() < 2) {
      return;
    }

    const double travel = std::max(simulation_speed_, 0.0) * dt;
    if (travel <= 1.0e-6) {
      return;
    }

    size_t closest_idx = 0;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < last_tracking_path_.poses.size(); ++i) {
      const auto & candidate = last_tracking_path_.poses[i];
      const double dist = distance2d(
        simulated_pose_.pose.position.x, simulated_pose_.pose.position.y,
        candidate.pose.position.x, candidate.pose.position.y);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_idx = i;
      }
    }

    geometry_msgs::msg::PoseStamped target_pose = last_tracking_path_.poses.back();
    double remaining = travel;
    for (size_t i = closest_idx + 1; i < last_tracking_path_.poses.size(); ++i) {
      const auto & prev = i == closest_idx + 1 ? simulated_pose_ : last_tracking_path_.poses[i - 1];
      const auto & next = last_tracking_path_.poses[i];
      const double segment = distance2d(
        prev.pose.position.x, prev.pose.position.y,
        next.pose.position.x, next.pose.position.y);
      if (segment <= 1.0e-6) {
        continue;
      }
      if (remaining <= segment) {
        const double ratio = remaining / segment;
        target_pose = prev;
        target_pose.pose.position.x =
          prev.pose.position.x + ratio * (next.pose.position.x - prev.pose.position.x);
        target_pose.pose.position.y =
          prev.pose.position.y + ratio * (next.pose.position.y - prev.pose.position.y);
        const double yaw = std::atan2(
          next.pose.position.y - prev.pose.position.y,
          next.pose.position.x - prev.pose.position.x);
        target_pose.pose.orientation = yawToQuaternion(yaw);
        break;
      }
      remaining -= segment;
    }

    simulated_pose_.pose = target_pose.pose;
    if (distance2d(
        simulated_pose_.pose.position.x, simulated_pose_.pose.position.y,
        default_goal_x_, default_goal_y_) < 0.05)
    {
      playback_enabled_ = false;
    }
    simulated_pose_.header.stamp = now();
  }

  void buildStraightGlobalPath()
  {
    const auto start = robotPose();
    const double goal_x = goal_pose_.header.frame_id.empty() ?
      default_goal_x_ : goal_pose_.pose.position.x;
    const double goal_y = goal_pose_.header.frame_id.empty() ?
      default_goal_y_ : goal_pose_.pose.position.y;
    const double goal_yaw = goal_pose_.header.frame_id.empty() ?
      default_goal_yaw_ : tf2::getYaw(goal_pose_.pose.orientation);

    const double length = distance2d(start.pose.position.x, start.pose.position.y, goal_x, goal_y);
    const size_t count = std::max<size_t>(
      2, static_cast<size_t>(std::ceil(length / std::max(straight_path_resolution_, 0.02))) + 1);

    global_path_ = nav_msgs::msg::Path();
    global_path_.header.frame_id = frame_id_;
    global_path_.header.stamp = now();
    global_path_.poses.reserve(count);

    for (size_t i = 0; i < count; ++i) {
      const double t = count <= 1 ? 1.0 : static_cast<double>(i) / static_cast<double>(count - 1);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = global_path_.header;
      pose.pose.position.x = start.pose.position.x + t * (goal_x - start.pose.position.x);
      pose.pose.position.y = start.pose.position.y + t * (goal_y - start.pose.position.y);
      const double yaw = i + 1 < count ?
        std::atan2(goal_y - start.pose.position.y, goal_x - start.pose.position.x) : goal_yaw;
      pose.pose.orientation = yawToQuaternion(yaw);
      global_path_.poses.push_back(pose);
    }
  }

  nav2_costmap_2d::Costmap2D buildCostmap(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto size_x = static_cast<unsigned int>(std::ceil(map_width_ / map_resolution_));
    const auto size_y = static_cast<unsigned int>(std::ceil(map_height_ / map_resolution_));
    const double origin_x = pose.pose.position.x - map_width_ * 0.5;
    const double origin_y = pose.pose.position.y - map_height_ * 0.5;
    nav2_costmap_2d::Costmap2D costmap(
      size_x, size_y, map_resolution_, origin_x, origin_y, nav2_costmap_2d::FREE_SPACE);

    for (const auto & obstacle : manual_obstacles_) {
      paintObstacle(costmap, obstacle, obstacle_radius_, nav2_costmap_2d::LETHAL_OBSTACLE);
      paintObstacle(costmap, obstacle, obstacle_inflation_radius_, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
    return costmap;
  }

  void paintObstacle(
    nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::Point & center,
    double radius,
    unsigned char cost)
  {
    const double resolution = costmap.getResolution();
    const int radius_cells = static_cast<int>(std::ceil(radius / resolution));
    unsigned int center_x = 0;
    unsigned int center_y = 0;
    if (!costmap.worldToMap(center.x, center.y, center_x, center_y)) {
      return;
    }

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int mx = static_cast<int>(center_x) + dx;
        const int my = static_cast<int>(center_y) + dy;
        if (mx < 0 || my < 0 ||
          mx >= static_cast<int>(costmap.getSizeInCellsX()) ||
          my >= static_cast<int>(costmap.getSizeInCellsY()))
        {
          continue;
        }
        double wx = 0.0;
        double wy = 0.0;
        costmap.mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
        if (distance2d(wx, wy, center.x, center.y) <= radius) {
          const auto old_cost = costmap.getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
          costmap.setCost(
            static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
            std::max(old_cost, cost));
        }
      }
    }
  }

  void updateOnce()
  {
    const auto pose = robotPose();
    if (global_path_.poses.empty()) {
      buildStraightGlobalPath();
    }
    auto costmap = buildCostmap(pose);
    const auto result = planner_->updateRoute(pose, global_path_, costmap);

    // Map SegmentBypassResult to local path variables:
    // active_route  → tracking_path (the route the robot should track)
    // bypass_path   → astar_path    (hybrid A* bypass around blocked segment)
    // smoothed_bypass → optimized_path (smoothed version of the bypass)
    nav_msgs::msg::Path tracking_path = result.active_route;
    nav_msgs::msg::Path astar_path = result.bypass_path;
    nav_msgs::msg::Path optimized_path = result.smoothed_bypass;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Local planner result: %s, bypass poses=%zu, smoothed poses=%zu, active route poses=%zu",
      result.message.c_str(), result.bypass_path.poses.size(), result.smoothed_bypass.poses.size(),
      result.active_route.poses.size());
    if (result.success && !result.active_route.poses.empty()) {
      stable_tracking_path_ = result.active_route;
      stable_astar_path_ = result.bypass_path;
      stable_path_valid_ = true;
      tracking_path = result.active_route;
    } else if (!result.route_changed) {
      // No blockage found — use active_route as-is
      tracking_path = result.active_route;
      if (tracking_path.poses.empty()) {
        tracking_path = prunePathFromRobot(global_path_, pose);
      }
    } else {
      // Planner failed, fall back to stable path or pruned global path
      nav_msgs::msg::Path reused_path;
      if (stable_path_reuse_enabled_ && tryReuseStablePath(pose, costmap, reused_path)) {
        tracking_path = reused_path;
        stable_tracking_path_ = reused_path;
      } else {
        tracking_path = prunePathFromRobot(global_path_, pose);
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Local planner failed: %s. Using fallback path, remaining length %.2f m",
        result.message.c_str(), pathLength(tracking_path));
    }

    last_tracking_path_ = tracking_path;
    auto optimized_or_tracking = optimized_path;
    if (optimized_or_tracking.poses.empty()) {
      optimized_or_tracking = tracking_path;
    }

    publishPath(reference_path_pub_, global_path_);
    publishPath(astar_path_pub_, astar_path);
    publishPath(optimized_path_pub_, optimized_or_tracking);
    publishPath(tracking_path_pub_, tracking_path);
    publishTestMap(costmap);
    publishObstacleMarkers();
    publishCostmapMarker(costmap);
    publishStartGoalMarkers(pose);
  }

  bool tryReuseStablePath(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav2_costmap_2d::Costmap2D & costmap,
    nav_msgs::msg::Path & reused_path) const
  {
    if (!stable_path_valid_ || stable_tracking_path_.poses.size() < 2) {
      return false;
    }
    reused_path = prunePathFromRobot(stable_tracking_path_, robot_pose);
    if (reused_path.poses.size() < 2) {
      return false;
    }
    if (distance2d(
        robot_pose.pose.position.x, robot_pose.pose.position.y,
        reused_path.poses.front().pose.position.x, reused_path.poses.front().pose.position.y) >
      stable_path_max_robot_distance_)
    {
      return false;
    }
    if (pathLength(reused_path) < stable_path_min_remaining_length_) {
      return false;
    }
    return isPathTraversable(reused_path, costmap);
  }

  bool isPathTraversable(
    const nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D & costmap) const
  {
    if (path.poses.size() < 2) {
      return false;
    }

    const double step = std::max(costmap.getResolution() * 0.5, 0.01);
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & start = path.poses[i - 1].pose.position;
      const auto & end = path.poses[i].pose.position;
      const double length = distance2d(start.x, start.y, end.x, end.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(length / step)));
      for (int j = 0; j <= steps; ++j) {
        const double t = static_cast<double>(j) / static_cast<double>(steps);
        const double wx = start.x + t * (end.x - start.x);
        const double wy = start.y + t * (end.y - start.y);
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap.worldToMap(wx, wy, mx, my)) {
          continue;
        }
        const unsigned char cost = costmap.getCost(mx, my);
        if (cost != nav2_costmap_2d::NO_INFORMATION && cost >= options_.astar_lethal_cost) {
          return false;
        }
      }
    }
    return true;
  }

  nav_msgs::msg::Path prunePathFromRobot(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose) const
  {
    nav_msgs::msg::Path pruned;
    pruned.header = path.header;
    pruned.header.stamp = now();
    if (path.poses.empty()) {
      return pruned;
    }

    size_t closest_idx = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const double dist = distance2d(
        robot_pose.pose.position.x, robot_pose.pose.position.y,
        path.poses[i].pose.position.x, path.poses[i].pose.position.y);
      if (dist < best_dist) {
        best_dist = dist;
        closest_idx = i;
      }
    }

    auto robot_anchor = robot_pose;
    robot_anchor.header = pruned.header;
    pruned.poses.push_back(robot_anchor);
    for (size_t i = closest_idx; i < path.poses.size(); ++i) {
      auto pose = path.poses[i];
      pose.header = pruned.header;
      if (distance2d(
          pose.pose.position.x, pose.pose.position.y,
          pruned.poses.back().pose.position.x, pruned.poses.back().pose.position.y) > 0.02)
      {
        pruned.poses.push_back(pose);
      }
    }
    return pruned;
  }

  double pathLength(const nav_msgs::msg::Path & path) const
  {
    double length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      length += distance2d(
        path.poses[i - 1].pose.position.x, path.poses[i - 1].pose.position.y,
        path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    }
    return length;
  }

  void publishPath(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
    nav_msgs::msg::Path path)
  {
    path.header.frame_id = path.header.frame_id.empty() ? frame_id_ : normalizeFrame(path.header.frame_id);
    path.header.stamp = now();
    for (auto & pose : path.poses) {
      pose.header.frame_id = path.header.frame_id;
      pose.header.stamp = path.header.stamp;
    }
    publisher->publish(path);
  }

  void publishObstacleMarkers()
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id_;
    delete_all.header.stamp = now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(delete_all);

    for (size_t i = 0; i < manual_obstacles_.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "manual_obstacles";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = manual_obstacles_[i];
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_radius_ * 2.0;
      marker.scale.y = obstacle_radius_ * 2.0;
      marker.scale.z = 0.08;
      marker.color.r = 1.0;
      marker.color.g = 0.1;
      marker.color.b = 0.05;
      marker.color.a = 0.9;
      array.markers.push_back(marker);
    }

    obstacle_marker_pub_->publish(array);
  }

  void publishCostmapMarker(const nav2_costmap_2d::Costmap2D & costmap)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "test_costmap_obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = costmap.getResolution();
    marker.scale.y = costmap.getResolution();
    marker.scale.z = 0.03;
    marker.color.r = 0.9;
    marker.color.g = 0.2;
    marker.color.b = 0.0;
    marker.color.a = 0.35;

    for (unsigned int y = 0; y < costmap.getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < costmap.getSizeInCellsX(); ++x) {
        if (costmap.getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          geometry_msgs::msg::Point point;
          costmap.mapToWorld(x, y, point.x, point.y);
          point.z = 0.015;
          marker.points.push_back(point);
        }
      }
    }
    costmap_marker_pub_->publish(marker);
  }

  void publishTestMap(const nav2_costmap_2d::Costmap2D & costmap)
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = frame_id_;
    map.header.stamp = now();
    map.info.resolution = costmap.getResolution();
    map.info.width = costmap.getSizeInCellsX();
    map.info.height = costmap.getSizeInCellsY();
    map.info.origin.position.x = costmap.getOriginX();
    map.info.origin.position.y = costmap.getOriginY();
    map.info.origin.orientation.w = 1.0;
    map.data.resize(static_cast<size_t>(map.info.width) * static_cast<size_t>(map.info.height), 0);

    for (unsigned int y = 0; y < costmap.getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < costmap.getSizeInCellsX(); ++x) {
        const auto cost = costmap.getCost(x, y);
        int8_t value = 0;
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          value = 100;
        } else if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          value = 70;
        }
        map.data[static_cast<size_t>(y) * map.info.width + x] = value;
      }
    }
    test_map_pub_->publish(map);
  }

  void publishStartGoalMarkers(const geometry_msgs::msg::PoseStamped & start)
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id_;
    delete_all.header.stamp = now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(delete_all);

    const auto make_marker =
      [this](int id, const std::string & ns, const geometry_msgs::msg::PoseStamped & pose,
      float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose.pose;
        marker.pose.position.z = 0.05;
        marker.scale.x = 0.35;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        return marker;
      };

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = frame_id_;
    goal.header.stamp = now();
    goal.pose.position.x = goal_pose_.header.frame_id.empty() ? default_goal_x_ : goal_pose_.pose.position.x;
    goal.pose.position.y = goal_pose_.header.frame_id.empty() ? default_goal_y_ : goal_pose_.pose.position.y;
    goal.pose.orientation = goal_pose_.header.frame_id.empty() ?
      yawToQuaternion(default_goal_yaw_) : goal_pose_.pose.orientation;

    array.markers.push_back(make_marker(0, "start", start, 0.1F, 0.8F, 0.2F));
    array.markers.push_back(make_marker(1, "goal", goal, 0.1F, 0.25F, 1.0F));
    start_goal_marker_pub_->publish(array);
  }

  std::string frame_id_;
  std::string robot_frame_;
  bool use_tf_pose_{true};
  bool use_nav2_plan_{true};
  double map_width_{6.0};
  double map_height_{6.0};
  double map_resolution_{0.05};
  double obstacle_radius_{0.12};
  double obstacle_inflation_radius_{0.18};
  double publish_frequency_{5.0};
  double current_speed_{0.30};
  double default_start_x_{-2.0};
  double default_start_y_{-2.0};
  double default_start_yaw_{0.0};
  double default_goal_x_{2.0};
  double default_goal_y_{2.0};
  double default_goal_yaw_{0.0};
  double straight_path_resolution_{0.05};
  bool playback_enabled_{false};
  double simulation_speed_{0.20};
  bool stable_path_reuse_enabled_{true};
  double stable_path_max_robot_distance_{0.35};
  double stable_path_min_remaining_length_{0.40};

  origincar_navigation::local_avoidance::SegmentBypassOptions options_;
  std::unique_ptr<origincar_navigation::local_avoidance::SegmentBypassPlanner> planner_;
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path last_tracking_path_;
  nav_msgs::msg::Path stable_tracking_path_;
  nav_msgs::msg::Path stable_astar_path_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped simulated_pose_;
  rclcpp::Time last_update_time_;
  bool stable_path_valid_{false};
  std::vector<geometry_msgs::msg::Point> manual_obstacles_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr nav_plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr play_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr astar_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tracking_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr costmap_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr start_goal_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr test_map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalAvoidanceManualTestNode>());
  rclcpp::shutdown();
  return 0;
}
