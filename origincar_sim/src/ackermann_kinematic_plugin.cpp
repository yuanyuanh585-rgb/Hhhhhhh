// Gazebo Fortress System plugin for the Origincar Ackermann chassis.
//
// The plugin keeps the same ROS interface as the previous Gazebo Classic plugin:
//   - subscribes to /cmd_vel and /ackermann_cmd
//   - publishes /odom and odom -> base_footprint
//   - updates the simulated model with a simple Ackermann kinematic model
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace origincar_sim
{
using ClockDuration = std::chrono::steady_clock::duration;
namespace gzsim = ignition::gazebo;

class AckermannKinematicPlugin final
  : public gzsim::System,
    public gzsim::ISystemConfigure,
    public gzsim::ISystemPreUpdate
{
public:
  AckermannKinematicPlugin() = default;

  ~AckermannKinematicPlugin() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void Configure(
    const gzsim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gzsim::EntityComponentManager & ecm,
    gzsim::EventManager &) override
  {
    model_ = gzsim::Model(entity);
    if (!model_.Valid(ecm)) {
      return;
    }

    cmd_vel_topic_ = GetSdfString(sdf, "cmd_vel_topic", "/cmd_vel");
    ackermann_topic_ = GetSdfString(sdf, "ackermann_topic", "/ackermann_cmd");
    odom_topic_ = GetSdfString(sdf, "odom_topic", "/odom");
    odom_frame_ = GetSdfString(sdf, "odom_frame", "odom");
    base_frame_ = GetSdfString(sdf, "base_frame", "base_footprint");
    wheelbase_ = GetSdfDouble(sdf, "wheelbase", 0.189);
    track_width_ = GetSdfDouble(sdf, "track_width", 0.1682);
    wheel_radius_ = GetSdfDouble(sdf, "wheel_radius", 0.03);
    max_steering_angle_ = GetSdfDouble(sdf, "max_steering_angle", 0.65);
    command_timeout_ = GetSdfDouble(sdf, "command_timeout", 0.5);
    model_z_ = GetSdfDouble(sdf, "model_z", 0.0);

    front_left_steer_joint_ = JointByName(ecm, GetSdfString(sdf, "front_left_steer_joint", "front_left_steer_joint"));
    front_right_steer_joint_ = JointByName(ecm, GetSdfString(sdf, "front_right_steer_joint", "front_right_steer_joint"));
    front_left_wheel_joint_ = JointByName(ecm, GetSdfString(sdf, "front_left_wheel_joint", "front_left_wheel_joint"));
    front_right_wheel_joint_ = JointByName(ecm, GetSdfString(sdf, "front_right_wheel_joint", "front_right_wheel_joint"));
    rear_left_wheel_joint_ = JointByName(ecm, GetSdfString(sdf, "rear_left_wheel_joint", "rear_left_wheel_joint"));
    rear_right_wheel_joint_ = JointByName(ecm, GetSdfString(sdf, "rear_right_wheel_joint", "rear_right_wheel_joint"));

    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("origincar_ackermann_kinematic");
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&AckermannKinematicPlugin::OnCmdVel, this, std::placeholders::_1));
    ackermann_sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      ackermann_topic_, rclcpp::QoS(10),
      std::bind(&AckermannKinematicPlugin::OnAckermann, this, std::placeholders::_1));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    RCLCPP_INFO(node_->get_logger(), "Origincar Ackermann kinematic plugin loaded for Gazebo Fortress");
  }

  void PreUpdate(const gzsim::UpdateInfo & info, gzsim::EntityComponentManager & ecm) override
  {
    if (info.paused || !model_.Valid(ecm)) {
      return;
    }

    const auto current_time = info.simTime;
    if (!have_last_update_time_) {
      const auto pose = gzsim::worldPose(model_.Entity(), ecm);
      x_ = pose.Pos().X();
      y_ = pose.Pos().Y();
      yaw_ = pose.Rot().Yaw();
      last_update_time_ = current_time;
      last_command_time_ = current_time;
      have_last_update_time_ = true;
      return;
    }

    const double dt = std::chrono::duration<double>(current_time - last_update_time_).count();
    if (dt <= 0.0) {
      return;
    }
    last_update_time_ = current_time;

    double speed = 0.0;
    double steering = 0.0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_received_) {
        last_command_time_ = current_time;
        command_received_ = false;
      }
      if (std::chrono::duration<double>(current_time - last_command_time_).count() <= command_timeout_) {
        speed = linear_speed_;
        steering = steering_angle_;
      }
    }

    const double yaw_rate = std::abs(wheelbase_) > 1e-6 ? speed * std::tan(steering) / wheelbase_ : 0.0;
    x_ += speed * std::cos(yaw_) * dt;
    y_ += speed * std::sin(yaw_) * dt;
    yaw_ = NormalizeAngle(yaw_ + yaw_rate * dt);
    wheel_angle_ += speed / std::max(wheel_radius_, 1e-6) * dt;

    const ignition::math::Pose3d pose(x_, y_, model_z_, 0.0, 0.0, yaw_);
    model_.SetWorldPoseCmd(ecm, pose);
    ecm.SetComponentData<gzsim::components::WorldLinearVelocityCmd>(
      model_.Entity(), ignition::math::Vector3d(speed * std::cos(yaw_), speed * std::sin(yaw_), 0.0));
    ecm.SetComponentData<gzsim::components::WorldAngularVelocityCmd>(
      model_.Entity(), ignition::math::Vector3d(0.0, 0.0, yaw_rate));

    UpdateJoints(ecm, steering);
    PublishOdometry(current_time, speed, yaw_rate);
  }

private:
  static std::string GetSdfString(
    const std::shared_ptr<const sdf::Element> & sdf,
    const std::string & name,
    const std::string & fallback)
  {
    return sdf && sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
  }

  static double GetSdfDouble(
    const std::shared_ptr<const sdf::Element> & sdf,
    const std::string & name,
    double fallback)
  {
    return sdf && sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

  gzsim::Joint JointByName(gzsim::EntityComponentManager & ecm, const std::string & name)
  {
    const auto entity = model_.JointByName(ecm, name);
    if (entity == gzsim::kNullEntity && node_) {
      RCLCPP_WARN(node_->get_logger(), "Joint [%s] was not found in Gazebo model", name.c_str());
    }
    return gzsim::Joint(entity);
  }

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    linear_speed_ = msg->linear.x;
    steering_angle_ = 0.0;
    if (std::abs(msg->linear.x) > 1e-4) {
      steering_angle_ = std::atan(wheelbase_ * msg->angular.z / msg->linear.x);
    }
    steering_angle_ = ClampSteering(steering_angle_);
    command_received_ = true;
  }

  void OnAckermann(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    linear_speed_ = msg->drive.speed;
    steering_angle_ = ClampSteering(msg->drive.steering_angle);
    command_received_ = true;
  }

  double ClampSteering(double steering) const
  {
    return std::clamp(steering, -max_steering_angle_, max_steering_angle_);
  }

  static double NormalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  void UpdateJoints(gzsim::EntityComponentManager & ecm, double center_steering)
  {
    double left_steering = center_steering;
    double right_steering = center_steering;
    if (std::abs(center_steering) > 1e-4) {
      const double radius = wheelbase_ / std::tan(std::abs(center_steering));
      const double inner = std::atan(wheelbase_ / std::max(radius - track_width_ / 2.0, 1e-4));
      const double outer = std::atan(wheelbase_ / (radius + track_width_ / 2.0));
      if (center_steering > 0.0) {
        left_steering = inner;
        right_steering = outer;
      } else {
        left_steering = -outer;
        right_steering = -inner;
      }
    }

    ResetJointPosition(ecm, front_left_steer_joint_, left_steering);
    ResetJointPosition(ecm, front_right_steer_joint_, right_steering);
    ResetJointPosition(ecm, front_left_wheel_joint_, wheel_angle_);
    ResetJointPosition(ecm, front_right_wheel_joint_, wheel_angle_);
    ResetJointPosition(ecm, rear_left_wheel_joint_, wheel_angle_);
    ResetJointPosition(ecm, rear_right_wheel_joint_, wheel_angle_);
  }

  static void ResetJointPosition(gzsim::EntityComponentManager & ecm, gzsim::Joint & joint, double position)
  {
    if (joint.Entity() != gzsim::kNullEntity) {
      joint.ResetPosition(ecm, {position});
    }
  }

  void PublishOdometry(ClockDuration sim_time, double speed, double yaw_rate)
  {
    const auto stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count();
    rclcpp::Time stamp(stamp_ns);

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();
    odom.twist.twist.linear.x = speed;
    odom.twist.twist.angular.z = yaw_rate;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  gzsim::Model model_;
  gzsim::Joint front_left_steer_joint_;
  gzsim::Joint front_right_steer_joint_;
  gzsim::Joint front_left_wheel_joint_;
  gzsim::Joint front_right_wheel_joint_;
  gzsim::Joint rear_left_wheel_joint_;
  gzsim::Joint rear_right_wheel_joint_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex command_mutex_;
  std::string cmd_vel_topic_;
  std::string ackermann_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double wheelbase_{0.189};
  double track_width_{0.1682};
  double wheel_radius_{0.03};
  double max_steering_angle_{0.65};
  double command_timeout_{0.5};
  double linear_speed_{0.0};
  double steering_angle_{0.0};
  double x_{0.0};
  double y_{0.0};
  double model_z_{0.0};
  double yaw_{0.0};
  double wheel_angle_{0.0};
  bool command_received_{false};
  bool have_last_update_time_{false};
  ClockDuration last_update_time_{};
  ClockDuration last_command_time_{};
};
}  // namespace origincar_sim

IGNITION_ADD_PLUGIN(
  origincar_sim::AckermannKinematicPlugin,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  origincar_sim::AckermannKinematicPlugin,
  "origincar_sim::AckermannKinematicPlugin")

IGNITION_ADD_PLUGIN_ALIAS(
  origincar_sim::AckermannKinematicPlugin,
  "origincar_ackermann_kinematic")
