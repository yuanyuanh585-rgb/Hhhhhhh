#ifndef ORIGINCAR_NAVIGATION__LOCAL_AVOIDANCE__SEGMENT_BYPASS_PLANNER_HPP_
#define ORIGINCAR_NAVIGATION__LOCAL_AVOIDANCE__SEGMENT_BYPASS_PLANNER_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace origincar_navigation
{
namespace local_avoidance
{

struct SegmentBypassOptions
{
  bool enabled{true};
  double replan_frequency{3.0};
  double scan_distance{2.0};
  double pre_margin{0.30};
  double post_margin{0.50};
  int pre_point_margin{3};
  int post_point_margin{5};
  double min_blocked_length{0.05};
  double min_replace_length{0.40};
  double prune_search_distance{1.0};
  bool smoothing_enabled{true};
  bool smoothing_collision_check{true};

  bool astar_allow_unknown{false};
  unsigned char astar_lethal_cost{253};
  double astar_cost_weight{0.02};
  int astar_grid_downsample_factor{1};

  double hybrid_wheelbase{0.189};
  double hybrid_max_steering_angle{0.65};
  int hybrid_yaw_bins{72};
  int hybrid_steering_samples{5};
  double hybrid_step_distance{0.12};
  double hybrid_goal_tolerance{0.12};
  double hybrid_yaw_tolerance{0.70};
  int hybrid_max_iterations{20000};
  double hybrid_steering_cost_weight{0.05};
  double hybrid_goal_yaw_weight{0.05};
  double hybrid_max_path_length_ratio{3.0};
  double hybrid_search_lateral_margin{0.80};

  double sample_interval{0.05};
  double route_sample_interval{0.05};
};

struct SegmentBypassResult
{
  bool success{false};
  bool route_changed{false};
  bool blocked_segment_found{false};
  bool used_smoothed_bypass{false};
  std::string message;
  nav_msgs::msg::Path active_route;
  nav_msgs::msg::Path blocked_segment;
  nav_msgs::msg::Path bypass_path;
  nav_msgs::msg::Path smoothed_bypass;
};

class SegmentBypassPlanner
{
public:
  struct Point2
  {
    double x{0.0};
    double y{0.0};
  };

  explicit SegmentBypassPlanner(SegmentBypassOptions options);

  void setOptions(const SegmentBypassOptions & options);

  SegmentBypassResult updateRoute(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & active_route,
    const nav2_costmap_2d::Costmap2D & costmap) const;

private:
  struct GridCell
  {
    unsigned int x{0};
    unsigned int y{0};
  };

  struct BlockedSegment
  {
    size_t first_idx{0};
    size_t last_idx{0};
    bool valid{false};
  };

  nav_msgs::msg::Path pruneRouteFromRobot(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & route) const;

  BlockedSegment findFirstBlockedSegment(
    const nav_msgs::msg::Path & route,
    const nav2_costmap_2d::Costmap2D & costmap) const;

  bool isWorldPointBlocked(
    double wx,
    double wy,
    const nav2_costmap_2d::Costmap2D & costmap) const;

  bool isCellTraversable(
    const nav2_costmap_2d::Costmap2D & costmap,
    unsigned int x,
    unsigned int y) const;

  bool isDownsampledCellTraversable(
    const nav2_costmap_2d::Costmap2D & costmap,
    unsigned int cell_x,
    unsigned int cell_y,
    unsigned int downsample_factor) const;

  bool findTraversableRouteCell(
    const nav2_costmap_2d::Costmap2D & costmap,
    const nav_msgs::msg::Path & route,
    size_t seed_idx,
    int direction,
    size_t & route_idx,
    GridCell & cell) const;

  size_t indexBeforeDistance(
    const nav_msgs::msg::Path & route,
    size_t start_idx,
    double distance) const;

  size_t indexAfterDistance(
    const nav_msgs::msg::Path & route,
    size_t start_idx,
    double distance) const;

  size_t indexBeforePointCount(
    const nav_msgs::msg::Path & route,
    size_t start_idx,
    int point_count) const;

  size_t indexAfterPointCount(
    const nav_msgs::msg::Path & route,
    size_t start_idx,
    int point_count) const;

  double pathLength(
    const nav_msgs::msg::Path & route,
    size_t start_idx,
    size_t end_idx) const;

  double pathLength(const nav_msgs::msg::Path & route) const;

  nav_msgs::msg::Path runHybridAstar(
    const GridCell & start,
    const GridCell & goal,
    const Point2 & start_anchor,
    const Point2 & goal_anchor,
    double start_yaw,
    double goal_yaw,
    double reference_path_length,
    const nav2_costmap_2d::Costmap2D & costmap,
    const std::string & frame_id) const;

  nav_msgs::msg::Path smoothBypassPath(
    const nav_msgs::msg::Path & bypass_path,
    const nav_msgs::msg::Path & route,
    size_t replace_start_idx,
    size_t replace_end_idx,
    const nav2_costmap_2d::Costmap2D & costmap) const;

  nav_msgs::msg::Path replaceRouteSegment(
    const nav_msgs::msg::Path & route,
    size_t replace_start_idx,
    size_t replace_end_idx,
    const nav_msgs::msg::Path & replacement) const;

  bool isPathCollisionFree(
    const nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D & costmap) const;

  std::vector<Point2> pathToPoints(const nav_msgs::msg::Path & path) const;

  nav_msgs::msg::Path pointsToPath(
    const std::vector<Point2> & points,
    const std::string & frame_id) const;

  std::vector<Point2> resamplePolyline(
    const std::vector<Point2> & points,
    double interval) const;

  void updatePathYaws(nav_msgs::msg::Path & path) const;

  SegmentBypassOptions options_;
};

}  // namespace local_avoidance
}  // namespace origincar_navigation

#endif  // ORIGINCAR_NAVIGATION__LOCAL_AVOIDANCE__SEGMENT_BYPASS_PLANNER_HPP_
