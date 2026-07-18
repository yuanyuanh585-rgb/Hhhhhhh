#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "angles/angles.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

namespace origincar_strategy
{
namespace
{

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

constexpr double kDynamicReverseGoalCode = -1.0;

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

double readRequiredDouble(
  const YAML::Node & node,
  const std::string & field_name,
  const std::string & route_name,
  size_t point_index)
{
  if (!node[field_name]) {
    throw std::runtime_error(
            "Missing '" + field_name + "' in " + route_name +
            " point " + std::to_string(point_index));
  }
  return node[field_name].as<double>();
}

geometry_msgs::msg::Pose parseRoutePoint(
  const YAML::Node & point_node,
  const std::string & route_name,
  size_t point_index)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = readRequiredDouble(point_node, "x", route_name, point_index);
  pose.position.y = readRequiredDouble(point_node, "y", route_name, point_index);
  pose.position.z = 0.0;

  if (point_node["orientation"]) {
    const auto orientation = point_node["orientation"];
    pose.orientation.x = readRequiredDouble(orientation, "x", route_name, point_index);
    pose.orientation.y = readRequiredDouble(orientation, "y", route_name, point_index);
    pose.orientation.z = readRequiredDouble(orientation, "z", route_name, point_index);
    pose.orientation.w = readRequiredDouble(orientation, "w", route_name, point_index);
  } else if (point_node["yaw_deg"]) {
    pose.orientation = quaternionFromYaw(point_node["yaw_deg"].as<double>() * M_PI / 180.0);
  } else if (point_node["yaw"]) {
    pose.orientation = quaternionFromYaw(point_node["yaw"].as<double>());
  } else {
    throw std::runtime_error(
            "Point " + std::to_string(point_index) + " in " + route_name +
            " must define orientation quaternion, yaw_deg, or yaw");
  }

  return pose;
}

}  // namespace

class RouteStrategyController : public rclcpp::Node
{
public:
  RouteStrategyController()
  : Node("route_strategy_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();
    loadRoutes();

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "route_strategy/start", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) { onStart(); });
    switch_branch_sub_ = create_subscription<std_msgs::msg::Int32>(
      "route_strategy/switch_branch", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) { onSwitchBranch(msg->data); });
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
      "route_strategy/cancel", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) { onCancel(); });

    state_pub_ = create_publisher<std_msgs::msg::String>("route_strategy/state", 10);
    active_route_pub_ = create_publisher<std_msgs::msg::Int32>("route_strategy/active_route", 10);

    timer_ = create_wall_timer(100ms, [this]() { onTimer(); });
    publishState();

    RCLCPP_INFO(get_logger(), "Route strategy controller is ready");
  }

private:
  enum class State
  {
    IDLE,
    ROUTE_1,
    WAITING_SWITCH,
    TRANSITION,
    ROUTE_SEQUENCE,
    COMPLETED,
    FAILED
  };

  enum class Direction
  {
    CLOCKWISE,
    COUNTERCLOCKWISE
  };

  struct RouteInfo
  {
    geometry_msgs::msg::Pose first;
    geometry_msgs::msg::Pose last;
  };

  void declareParameters()
  {
    const std::string default_routes_file =
      ament_index_cpp::get_package_share_directory("origincar_strategy") +
      "/config/global_routes.yaml";

    declare_parameter("routes_file", default_routes_file);
    declare_parameter("waiting_switch_timeout", 2.0);
    declare_parameter("default_direction", "clockwise");
    declare_parameter("transition_back_distance", 0.45);
    declare_parameter("transition_skip_angle_threshold_deg", 65.0);
    declare_parameter("global_frame", "map");
    declare_parameter("robot_base_frame", "base_footprint");
    declare_parameter("action_server_timeout", 5.0);
  }

  void loadParameters()
  {
    get_parameter("routes_file", routes_file_);
    get_parameter("waiting_switch_timeout", waiting_switch_timeout_);
    std::string default_direction_name;
    get_parameter("default_direction", default_direction_name);
    get_parameter("transition_back_distance", transition_back_distance_);
    double transition_skip_angle_threshold_deg = 65.0;
    get_parameter("transition_skip_angle_threshold_deg", transition_skip_angle_threshold_deg);
    get_parameter("global_frame", global_frame_);
    get_parameter("robot_base_frame", robot_base_frame_);
    get_parameter("action_server_timeout", action_server_timeout_);

    if (routes_file_.empty()) {
      routes_file_ =
        ament_index_cpp::get_package_share_directory("origincar_strategy") +
        "/config/global_routes.yaml";
    }
    const auto parsed_default_direction = directionFromName(default_direction_name);
    if (!parsed_default_direction) {
      RCLCPP_WARN(
        get_logger(), "default_direction must be clockwise/cw or counterclockwise/ccw, reset to clockwise");
      default_direction_ = Direction::CLOCKWISE;
    } else {
      default_direction_ = *parsed_default_direction;
    }
    waiting_switch_timeout_ = std::max(0.0, waiting_switch_timeout_);
    transition_back_distance_ = std::max(0.05, transition_back_distance_);
    transition_skip_angle_threshold_ = transition_skip_angle_threshold_deg * M_PI / 180.0;
    if (transition_skip_angle_threshold_ <= 0.0 || transition_skip_angle_threshold_ > M_PI) {
      RCLCPP_WARN(get_logger(), "transition_skip_angle_threshold_deg invalid, reset to 65");
      transition_skip_angle_threshold_ = 65.0 * M_PI / 180.0;
    }
    action_server_timeout_ = std::max(0.1, action_server_timeout_);
  }

  void loadRoutes()
  {
    YAML::Node config;
    try {
      config = YAML::LoadFile(routes_file_);
    } catch (const std::exception & ex) {
      throw std::runtime_error(
              "Failed to load route config file '" + routes_file_ + "': " + ex.what());
    }

    const auto routes_node = config["routes"];
    if (!routes_node || !routes_node.IsMap()) {
      throw std::runtime_error("routes section is missing or invalid in " + routes_file_);
    }

    for (const auto & item : routes_node) {
      const std::string route_key = item.first.as<std::string>();
      if (route_key.rfind("route_", 0) != 0) {
        continue;
      }

      int route_id = 0;
      try {
        route_id = std::stoi(route_key.substr(6));
      } catch (const std::exception &) {
        throw std::runtime_error("Invalid route name '" + route_key + "' in " + routes_file_);
      }

      const auto route_name = "route_" + std::to_string(route_id);
      const auto points_node = routes_node[route_name]["points"];
      if (!points_node || !points_node.IsSequence() || points_node.size() < 1) {
        throw std::runtime_error(route_name + ".points is missing or empty in " + routes_file_);
      }

      RouteInfo route;
      route.first = parseRoutePoint(points_node[0], route_name, 0);
      route.last = parseRoutePoint(points_node[points_node.size() - 1], route_name, points_node.size() - 1);
      routes_[route_id] = route;
    }

    if (routes_.find(1) == routes_.end()) {
      throw std::runtime_error("route_1.points is required in " + routes_file_);
    }
    validateRouteSequence(Direction::CLOCKWISE);
    validateRouteSequence(Direction::COUNTERCLOCKWISE);
  }

  std::optional<Direction> directionFromName(std::string direction_name) const
  {
    std::transform(
      direction_name.begin(), direction_name.end(), direction_name.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (direction_name == "clockwise" || direction_name == "cw" ||
      direction_name == "1" || direction_name == "2")
    {
      return Direction::CLOCKWISE;
    }
    if (direction_name == "counterclockwise" || direction_name == "counter_clockwise" ||
      direction_name == "counter-clockwise" || direction_name == "anticlockwise" ||
      direction_name == "ccw" || direction_name == "-1" || direction_name == "4")
    {
      return Direction::COUNTERCLOCKWISE;
    }

    return std::nullopt;
  }

  std::optional<Direction> directionFromSwitchValue(int value) const
  {
    if (value == 1 || value == 2) {
      return Direction::CLOCKWISE;
    }
    if (value == -1 || value == 4) {
      return Direction::COUNTERCLOCKWISE;
    }
    return std::nullopt;
  }

  const std::vector<int> & routeSequence(Direction direction) const
  {
    if (direction == Direction::CLOCKWISE) {
      return clockwise_route_sequence_;
    }
    return counterclockwise_route_sequence_;
  }

  std::string directionName(Direction direction) const
  {
    if (direction == Direction::CLOCKWISE) {
      return "CLOCKWISE";
    }
    return "COUNTERCLOCKWISE";
  }

  void validateRouteSequence(Direction direction) const
  {
    for (const int route_id : routeSequence(direction)) {
      if (routes_.find(route_id) == routes_.end()) {
        throw std::runtime_error(
                "route_" + std::to_string(route_id) + ".points is required for " +
                directionName(direction) + " in " + routes_file_);
      }
    }
  }

  void onStart()
  {
    if (state_ != State::IDLE && state_ != State::COMPLETED && state_ != State::FAILED) {
      RCLCPP_WARN(get_logger(), "Ignoring start because task is already running in %s", stateName().c_str());
      return;
    }

    locked_direction_.reset();
    active_route_sequence_.clear();
    active_route_sequence_index_ = 0;
    waiting_switch_started_.reset();
    active_route_id_ = 1;
    sendRouteGoal(1, State::ROUTE_1);
  }

  void onSwitchBranch(int switch_value)
  {
    const auto direction = directionFromSwitchValue(switch_value);
    if (!direction) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring switch_branch value %d, expected 1/2 for clockwise or -1/4 for counterclockwise",
        switch_value);
      return;
    }

    if (state_ == State::ROUTE_1) {
      locked_direction_ = *direction;
      RCLCPP_INFO(
        get_logger(), "Locked %s direction; transition will start after route_1",
        directionName(*locked_direction_).c_str());
      return;
    }

    if (state_ == State::WAITING_SWITCH) {
      locked_direction_ = *direction;
      startTransition();
      return;
    }

    if (state_ == State::TRANSITION || state_ == State::ROUTE_SEQUENCE) {
      RCLCPP_WARN(get_logger(), "Ignoring switch_branch after direction is locked");
      return;
    }

    RCLCPP_WARN(get_logger(), "Ignoring switch_branch while state is %s", stateName().c_str());
  }

  void onCancel()
  {
    cancelActiveGoal();
    setState(State::IDLE);
    active_route_id_ = 0;
    locked_direction_.reset();
    active_route_sequence_.clear();
    active_route_sequence_index_ = 0;
    waiting_switch_started_.reset();
    publishActiveRoute();
  }

  void onTimer()
  {
    if (!action_client_->action_server_is_ready()) {
      const auto now = steadyNow();
      if (!last_action_wait_log_ || (now - *last_action_wait_log_).seconds() > 2.0) {
        RCLCPP_WARN(get_logger(), "Waiting for navigate_to_pose action server");
        last_action_wait_log_ = now;
      }
      return;
    }

    if (state_ == State::WAITING_SWITCH && waiting_switch_started_) {
      const auto elapsed = (steadyNow() - *waiting_switch_started_).seconds();
      if (elapsed >= waiting_switch_timeout_) {
        locked_direction_ = default_direction_;
        RCLCPP_INFO(
          get_logger(), "WAITING_SWITCH timed out after %.2fs, using %s direction",
          waiting_switch_timeout_, directionName(*locked_direction_).c_str());
        startTransition();
      }
    }
  }

  rclcpp::Time steadyNow()
  {
    return steady_clock_.now();
  }

  void sendRouteGoal(int route_id, State next_state)
  {
    const auto route_it = routes_.find(route_id);
    if (route_it == routes_.end()) {
      failTask("route_" + std::to_string(route_id) + " is not loaded");
      return;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = global_frame_;
    goal_pose.header.stamp = now();
    goal_pose.pose = route_it->second.last;
    goal_pose.pose.position.z = static_cast<double>(route_id);

    sendNavigateGoal(goal_pose, next_state, route_id);
  }

  void startTransition()
  {
    if (!locked_direction_) {
      locked_direction_ = default_direction_;
    }

    if (!selectRouteSequence(*locked_direction_)) {
      return;
    }
    const int first_route_id = active_route_sequence_.front();

    waiting_switch_started_.reset();
    cancelActiveGoal();

    if (shouldSkipTransition(first_route_id)) {
      RCLCPP_INFO(
        get_logger(),
        "Target route_%d is within %.1f deg, skipping reverse transition",
        first_route_id, transition_skip_angle_threshold_ * 180.0 / M_PI);
      sendCurrentSequenceRoute();
      return;
    }

    const auto transition_goal = buildTransitionGoal(first_route_id);
    if (!transition_goal) {
      failTask("failed to build transition goal");
      return;
    }

    active_route_id_ = -1;
    sendNavigateGoal(*transition_goal, State::TRANSITION, -1);
  }

  bool selectRouteSequence(Direction direction)
  {
    active_route_sequence_ = routeSequence(direction);
    active_route_sequence_index_ = 0;

    if (active_route_sequence_.empty()) {
      failTask(directionName(direction) + " route sequence is empty");
      return false;
    }

    for (const int route_id : active_route_sequence_) {
      if (routes_.find(route_id) == routes_.end()) {
        failTask(
          "route_" + std::to_string(route_id) + " is not loaded for " +
          directionName(direction));
        return false;
      }
    }

    return true;
  }

  void sendCurrentSequenceRoute()
  {
    if (active_route_sequence_index_ >= active_route_sequence_.size()) {
      completeTask();
      return;
    }

    sendRouteGoal(active_route_sequence_[active_route_sequence_index_], State::ROUTE_SEQUENCE);
  }

  bool shouldSkipTransition(int branch_route_id)
  {
    const auto route_it = routes_.find(branch_route_id);
    if (route_it == routes_.end()) {
      return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Cannot evaluate transition skip angle: %s", ex.what());
      return false;
    }

    const double robot_x = transform.transform.translation.x;
    const double robot_y = transform.transform.translation.y;
    const double robot_yaw = tf2::getYaw(transform.transform.rotation);
    const auto & target = route_it->second.first.position;
    const double target_yaw = std::atan2(target.y - robot_y, target.x - robot_x);
    const double angle_error = std::fabs(
      angles::shortest_angular_distance(robot_yaw, target_yaw));
    return angle_error <= transition_skip_angle_threshold_;
  }

  std::optional<geometry_msgs::msg::PoseStamped> buildTransitionGoal(int branch_route_id)
  {
    const auto route_it = routes_.find(branch_route_id);
    if (route_it == routes_.end()) {
      RCLCPP_ERROR(get_logger(), "route_%d is not loaded", branch_route_id);
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to lookup %s -> %s transform: %s",
        global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = global_frame_;
    goal.header.stamp = now();
    goal.pose = route_it->second.first;
    goal.pose.position.z = kDynamicReverseGoalCode;
    return goal;
  }

  void sendNavigateGoal(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    State next_state,
    int active_route_id)
  {
    if (!action_client_->wait_for_action_server(std::chrono::duration<double>(action_server_timeout_))) {
      failTask("navigate_to_pose action server is not available");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;
    const int64_t goal_sequence = ++goal_sequence_;
    active_goal_sequence_ = goal_sequence;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, goal_sequence](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        if (goal_sequence != active_goal_sequence_) {
          return;
        }
        if (!goal_handle) {
          failTask("navigate_to_pose goal was rejected");
        } else {
          active_goal_handle_ = goal_handle;
        }
      };
    options.result_callback =
      [this, goal_sequence](const GoalHandleNavigateToPose::WrappedResult & result) {
        if (goal_sequence != active_goal_sequence_) {
          return;
        }
        active_goal_handle_.reset();
        handleNavigateResult(result);
      };

    active_route_id_ = active_route_id;
    setState(next_state);
    publishActiveRoute();
    action_client_->async_send_goal(goal, options);
  }

  void cancelActiveGoal()
  {
    if (active_goal_handle_) {
      action_client_->async_cancel_goal(active_goal_handle_);
      active_goal_handle_.reset();
    }
    ++active_goal_sequence_;
  }

  void handleNavigateResult(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_INFO(get_logger(), "Navigation goal was canceled");
      return;
    }

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      failTask("navigation goal failed in state " + stateName());
      return;
    }

    switch (state_) {
      case State::ROUTE_1:
        if (locked_direction_) {
          startTransition();
        } else {
          waiting_switch_started_ = steadyNow();
          active_route_id_ = 0;
          publishActiveRoute();
          setState(State::WAITING_SWITCH);
        }
        break;
      case State::TRANSITION:
        sendCurrentSequenceRoute();
        break;
      case State::ROUTE_SEQUENCE:
        ++active_route_sequence_index_;
        sendCurrentSequenceRoute();
        break;
      default:
        RCLCPP_WARN(get_logger(), "Unexpected navigation success in state %s", stateName().c_str());
        break;
    }
  }

  void completeTask()
  {
    active_route_id_ = 0;
    publishActiveRoute();
    setState(State::COMPLETED);
  }

  void failTask(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    active_route_id_ = 0;
    publishActiveRoute();
    setState(State::FAILED);
  }

  void setState(State state)
  {
    if (state_ == state) {
      publishState();
      return;
    }

    state_ = state;
    RCLCPP_INFO(get_logger(), "Route strategy state: %s", stateName().c_str());
    publishState();
  }

  std::string stateName() const
  {
    switch (state_) {
      case State::IDLE:
        return "IDLE";
      case State::ROUTE_1:
        return "ROUTE_1";
      case State::WAITING_SWITCH:
        return "WAITING_SWITCH";
      case State::TRANSITION:
        return "TRANSITION";
      case State::ROUTE_SEQUENCE:
        if (active_route_sequence_index_ < active_route_sequence_.size()) {
          return "ROUTE_" + std::to_string(active_route_sequence_[active_route_sequence_index_]);
        }
        return "ROUTE_SEQUENCE";
      case State::COMPLETED:
        return "COMPLETED";
      case State::FAILED:
        return "FAILED";
    }
    return "UNKNOWN";
  }

  void publishState()
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = stateName();
    state_pub_->publish(msg);
  }

  void publishActiveRoute()
  {
    if (!active_route_pub_) {
      return;
    }
    std_msgs::msg::Int32 msg;
    msg.data = active_route_id_;
    active_route_pub_->publish(msg);
  }

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr switch_branch_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr active_route_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_{State::IDLE};
  std::optional<Direction> locked_direction_;
  std::optional<rclcpp::Time> waiting_switch_started_;
  std::optional<rclcpp::Time> last_action_wait_log_;
  int active_route_id_{0};
  int64_t goal_sequence_{0};
  int64_t active_goal_sequence_{0};

  std::string routes_file_;
  double waiting_switch_timeout_{2.0};
  Direction default_direction_{Direction::CLOCKWISE};
  double transition_back_distance_{0.45};
  double transition_skip_angle_threshold_{65.0 * M_PI / 180.0};
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_footprint"};
  double action_server_timeout_{5.0};
  std::unordered_map<int, RouteInfo> routes_;
  std::vector<int> active_route_sequence_;
  size_t active_route_sequence_index_{0};
  const std::vector<int> clockwise_route_sequence_{2, 3};
  const std::vector<int> counterclockwise_route_sequence_{4, 5};
};

}  // namespace origincar_strategy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<origincar_strategy::RouteStrategyController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
