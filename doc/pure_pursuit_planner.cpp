#include "pure_pursuit_planner/pure_pursuit_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <Eigen/Core>
#include <cmath>
#include <angles/angles.h>
 
namespace pure_pursuit_planner {
 
// 构造函数：初始化所有成员变量 
PurePursuitPlanner::PurePursuitPlanner() 
    : tf_(nullptr),
      costmap_ros_(nullptr),
      initialized_(false),
      goal_reached_(false),
    //   last_filtered_vel_(0.0),
      is_reverse_mode_(false){
    current_velocity_ = 0;
    
}
 
// 析构函数 
PurePursuitPlanner::~PurePursuitPlanner() {}
 
/**
 * @brief 初始化规划器 
 * @param name 规划器命名空间 
 * @param tf TF缓冲区指针 
 * @param costmap_ros 代价地图指针 
 */
void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) {
        ROS_WARN("PurePursuitPlanner already initialized");
        return;
    }
 
    // 初始化核心组件 
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    ros::NodeHandle nh("~/" + name);
 
    // 加载参数（参数名，变量，默认值）
    nh.param("min_lookahead_distance",  min_lookahead_distance_, 0.3);
    nh.param("max_lookahead_distance",  max_lookahead_distance_, 2.0);
    nh.param("lookahead_ratio",  lookahead_ratio_, 0.5);
    nh.param("max_linear_velocity",  max_linear_velocity_, 0.5);
    nh.param("max_angular_",  max_angular_, 1.0);
    nh.param("goal_tolerance",  goal_tolerance_, 0.1);
    nh.param("decel_angle_threshold",  decel_angle_threshold_, 30.0);
    nh.param("decel_velocity_threshold",  decel_velocity_threshold_, 0.2);
    nh.param("decel_ratio",  decel_ratio_, 0.8);
    nh.param("max_angular_vel",  max_angular_vel_, 0.3);
    nh.param("wheelbase",  CARL_, 0.15);  // 轴距参数（前后轮中心距离）
    nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    // 新增倒车专用参数
    nh.param("reverse_mode",  is_reverse_mode_, false);  // 是否强制倒车模式
    nh.param("reverse_lookahead_ratio",  reverse_lookahead_ratio_, 0.4);  // 倒车前瞻距离系数 
    nh.param("max_reverse_velocity",  max_reverse_velocity_, 0.3);  // 最大倒车速度
    // 初始化发布器 
    local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan",  1);
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("target_pose",  1);
    goal_reached_pub_ = nh.advertise<std_msgs::Bool>("goal_reached",  1, true); // latch=true 
    last_goal_reached_status_ = false;
 
    // 初始化动态参数服务器 
    server_ = std::make_unique<dynamic_reconfigure::Server<PurePursuitConfig>>(nh);
    server_->setCallback(boost::bind(&PurePursuitPlanner::reconfigureCallback, this, _1, _2));
 
    initialized_ = true;
    ROS_INFO_STREAM("PurePursuitPlanner initialized with parameters:\n"
        << "  min_lookahead_distance: " << min_lookahead_distance_ << "m\n"
        << "  max_lookahead_distance: " << max_lookahead_distance_ << "m\n"
        << "  CARL_: " << CARL_ << "m\n"
        << "  goal_tolerance_: " << goal_tolerance_ << "m\n"
        << "  decel_angle_threshold: " << decel_angle_threshold_ << "rad");
}
 
/**
 * @brief 设置全局路径 
 * @param global_plan 全局路径（世界坐标系）
 * @return 是否设置成功 
 */
bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    if (!initialized_) {
        ROS_ERROR("PurePursuitPlanner not initialized");
        return false;
    }
 
    global_plan_ = global_plan;
    goal_reached_ = false;
    return true;
}
 
/**
 * @brief 计算速度指令（核心函数）
 * @param cmd_vel 输出的速度指令 
 * @return 是否计算成功 
 */
bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_ || global_plan_.empty()) {
        ROS_ERROR_THROTTLE(1.0, "Planner not ready or plan empty");
        return false;
    }
 
    // 获取当前机器人位姿（代价地图坐标系）
    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_ERROR_THROTTLE(1.0, "Cannot get robot pose");
        return false;
    }
 
    // 检查是否到达目标 
    if (isGoalReached()) {
        cmd_vel.linear.x  = 0;
        cmd_vel.angular.z  = 0;  
        current_velocity_ = 0; //当前速度
        goal_reached_ = true;
        return true;
    }
    //判断是否倒车
    is_reverse_mode_ = checkNeedReverse(robot_pose);
 
    // 计算动态前瞻距离 
    double lookahead_dist = getAdaptiveLookaheadDistance(current_velocity_);
 
    // 获取前瞻点 
    geometry_msgs::PoseStamped target_pose = getLookaheadPoint(robot_pose, lookahead_dist,true);
    publishLocalPlan(target_pose);  // 发布可视化 
 
    // 计算转向角度
    double angular_z = calculateSteering(target_pose, robot_pose);

 
    // 计算控制速度
    double linear_x = acalculateVel(target_pose,angular_z);
    
     // 设置速度指令（增加倒车处理）
    if (is_reverse_mode_) {
        cmd_vel.linear.x  = -linear_x;  // 倒车速度为负
        current_velocity_ = -linear_x;
        cmd_vel.angular.z  = -(linear_x * tan(angular_z * 0.0174533f)) / CARL_;  // 转向取反
    } else {
        cmd_vel.linear.x  = linear_x;
        current_velocity_ = linear_x;
        cmd_vel.angular.z  = (linear_x * tan(angular_z * 0.0174533f)) / CARL_;
    }
    // 设置初始速度指令 
    // cmd_vel.linear.x  = linear_x;
    // 转向角 转 角速度
    // cmd_vel.angular.z  =  (linear_x * tan(angular_z * 0.0174533f)) / CARL_;  // 0.0174533f = 度转弧度系数（π/180）
    // 记录本次控制时间 
    // last_time_ = ros::Time::now();
    return true;
}

// 新增函数：检查是否需要倒车
bool PurePursuitPlanner::checkNeedReverse(const geometry_msgs::PoseStamped& robot_pose) {

    //获取0.2米位置的点
    geometry_msgs::PoseStamped target_check_pose = getLookaheadPoint(robot_pose, 0.2,false);

    // 将目标点转换到机器人坐标系 
    geometry_msgs::PoseStamped target_in_robot;
    try {
        // 静态路径特殊处理：忽略时间戳 
        tf_->transform(target_check_pose, target_in_robot, robot_base_frame_, 
                    ros::Time(0),  // 特殊值表示最新可用变换 
                    target_check_pose.header.frame_id); 
    } catch (tf2::TransformException& ex) {
        ROS_WARN("TF error in steering calculation: %s", ex.what()); 
        return 0;
    }

    // 获取目标点在机器人坐标系中的坐标 
    double y = target_in_robot.pose.position.y;   // 横向偏移 
    double x = target_in_robot.pose.position.x;   // 前向距离 
    // ROS_INFO("前瞻点位置 x:%.2f",x);
    return x<0;
}


bool PurePursuitPlanner::arePosesEqual(const geometry_msgs::PoseStamped& pose1, 
                  const geometry_msgs::PoseStamped& pose2,
                  double position_tolerance,
                  double orientation_tolerance)
{
    // 比较位置 (x, y)
    if (std::abs(pose1.pose.position.x  - pose2.pose.position.x)  > position_tolerance ||
        std::abs(pose1.pose.position.y  - pose2.pose.position.y)  > position_tolerance)
    {
        return false;
    }
    
    // 比较四元数 (x, y, z, w)
    if (std::abs(pose1.pose.orientation.x  - pose2.pose.orientation.x)  > orientation_tolerance ||
        std::abs(pose1.pose.orientation.y  - pose2.pose.orientation.y)  > orientation_tolerance ||
        std::abs(pose1.pose.orientation.z  - pose2.pose.orientation.z)  > orientation_tolerance ||
        std::abs(pose1.pose.orientation.w  - pose2.pose.orientation.w)  > orientation_tolerance)
    {
        return false;
    }
    
    return true;
}
 
/**
 * @brief 检查是否到达目标 
 * @return 是否到达 
 */
bool PurePursuitPlanner::isGoalReached() {
    
    if(goal_reached_ == true){
        return true;
    }
    if (!initialized_ || global_plan_.empty()) {
        ROS_ERROR("Planner not ready or plan empty");
        return false;
    }
 
    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_ERROR("Failed to get robot pose");
        return false;
    }
 
    // 计算到终点的欧氏距离 
    double dx = global_plan_.back().pose.position.x  - robot_pose.pose.position.x; 
    double dy = global_plan_.back().pose.position.y  - robot_pose.pose.position.y; 
    double distance = hypot(dx, dy);
 
    bool current_status = (distance <goal_tolerance_);

    // // 校验角度
    if (current_status)
    {
      double currentpose_yaw = tf2::getYaw(robot_pose.pose.orientation);
      double targetpose_yaw = tf2::getYaw(global_plan_.back().pose.orientation);
      double diffangle = fabs(currentpose_yaw - targetpose_yaw);
      // printf("diffangle = %.2f,diffDistance = %.2f\n",diffangle,diffDistance);
      if (diffangle > 3.1415926)
        diffangle = 6.283 - diffangle;
      if (diffangle > 2.0)
      {
        current_status = false;
      }
    }
    // 状态变化时发布 
    if (current_status != last_goal_reached_status_) {
        std_msgs::Bool msg;
        msg.data  = current_status;
        goal_reached_pub_.publish(msg);
        last_goal_reached_status_ = current_status;
        ROS_INFO("Goal status changed to: %s", current_status ? "REACHED" : "NOT REACHED");
    }
    
    return current_status;
}
 
/**
 * @brief 动态参数回调 
 * @param config 新参数 
 * @param level 变更级别 
 */
void PurePursuitPlanner::reconfigureCallback(PurePursuitConfig& config, uint32_t level) {
    min_lookahead_distance_ = config.min_lookahead_distance; 
    max_lookahead_distance_ = config.max_lookahead_distance; 
    lookahead_ratio_ = config.lookahead_ratio; 
    max_linear_velocity_ = config.max_linear_velocity; 
    max_angular_ = config.max_angular_; 
    goal_tolerance_ = config.goal_tolerance; 
    decel_angle_threshold_ = config.decel_angle_threshold; 
    decel_velocity_threshold_ = config.decel_velocity_threshold; 
    decel_ratio_ = config.decel_ratio; 
    max_angular_vel_ = config.max_angular_vel; 
    CARL_ = config.wheelbase; 
}
 
/**
 * @brief 计算动态前瞻距离 
 * @param current_speed 当前线速度 
 * @return 前瞻距离 
 */
double PurePursuitPlanner::getAdaptiveLookaheadDistance(double current_speed) {
    double distance = min_lookahead_distance_ + lookahead_ratio_ * fabs(current_speed);
    if (is_reverse_mode_) {
        distance *= reverse_lookahead_ratio_;  // 倒车时缩短前瞻距离
    }
    return std::min(distance, max_lookahead_distance_);
}
 
/**
 * @brief 获取前瞻点 
 * @param robot_pose 机器人位姿 
 * @param lookahead_dist 前瞻距离 
 * @return 目标点位姿 
 */
geometry_msgs::PoseStamped PurePursuitPlanner::getLookaheadPoint(
    const geometry_msgs::PoseStamped& robot_pose, double lookahead_dist,bool need_extend) {
    
    // 查找路径上距离机器人最近的点 
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < global_plan_.size(); ++i) {
        double dx = global_plan_[i].pose.position.x  - robot_pose.pose.position.x; 
        double dy = global_plan_[i].pose.position.y  - robot_pose.pose.position.y; 
        double dist = hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
 
    // 从最近点开始搜索满足前瞻距离的点 
    for (size_t i = closest_idx; i < global_plan_.size(); ++i) {
        double dx = global_plan_[i].pose.position.x  - robot_pose.pose.position.x; 
        double dy = global_plan_[i].pose.position.y  - robot_pose.pose.position.y; 
        double dist = hypot(dx, dy);
        if (dist >= lookahead_dist) {
            // ROS_INFO("获取点位%d,距离%.2f",i,dist);
            return global_plan_[i];
        }
    }
    if(need_extend){
        // 如果没有满足条件的点，创建虚拟延长点用于位姿校准
        geometry_msgs::PoseStamped virtual_point = global_plan_.back();
        
        // 获取路径终点的朝向角度
        double goal_yaw = tf2::getYaw(virtual_point.pose.orientation);
        
        // 计算当前机器人到终点的距离
        double dx_to_goal = virtual_point.pose.position.x - robot_pose.pose.position.x;
        double dy_to_goal = virtual_point.pose.position.y - robot_pose.pose.position.y;
        double dist_to_goal = hypot(dx_to_goal, dy_to_goal);
        
        // 根据倒车模式调整虚拟点的朝向
        if (is_reverse_mode_) {
            goal_yaw += M_PI; // 倒车时角度翻转180度
            // 确保角度在[-π, π]范围内
            while (goal_yaw > M_PI) goal_yaw -= 2 * M_PI;
            while (goal_yaw < -M_PI) goal_yaw += 2 * M_PI;
        }
        
        // 计算需要延长的距离（确保前瞻点距离机器人至少为lookahead_dist）
        double extend_distance = lookahead_dist - dist_to_goal + 0.1; // 额外增加0.1m确保有足够的校准空间
        
        if (extend_distance > 0) {
            // 沿着终点的朝向方向创建虚拟点
            virtual_point.pose.position.x += extend_distance * cos(goal_yaw);
            virtual_point.pose.position.y += extend_distance * sin(goal_yaw);

            
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_yaw);
            virtual_point.pose.orientation = tf2::toMsg(q);
            // ROS_INFO("创建虚拟延长点，延长距离: %.2f", extend_distance);
        }
        
        return virtual_point;
    }else{
        return global_plan_.back();
    }
}
 
/**
 * @brief 计算转向指令（纯追踪核心算法）
 * @param target 目标点位姿 
 * @param robot_pose 机器人位姿 
 * @return 角速度指令(rad/s)
 */
double PurePursuitPlanner::calculateSteering(
    const geometry_msgs::PoseStamped& target, const geometry_msgs::PoseStamped& robot_pose) {
    
    // 将目标点转换到机器人坐标系 
    geometry_msgs::PoseStamped target_in_robot;
    try {
        tf_->transform(target, target_in_robot, robot_base_frame_, 
                      ros::Time(0),  // 特殊值表示最新可用变换 
                      target.header.frame_id); 
        // if (global_plan_.front().header.stamp  == ros::Time(0)) {
        // // 静态路径特殊处理：忽略时间戳 
        //     tf_->transform(target, target_in_robot, robot_base_frame_, 
        //               ros::Time(0),  // 特殊值表示最新可用变换 
        //               target.header.frame_id); 
        // } else {
        // // 动态路径正常处理 
        // // tf_->transform(target, target_in_robot, robot_pose.header.frame_id); 
        // }
    } catch (tf2::TransformException& ex) {
        ROS_WARN("TF error in steering calculation: %s", ex.what()); 
        return 0;
    }
    // ROS_INFO("target_in_robot fram_id:%s",robot_base_frame_);
    // 获取目标点在机器人坐标系中的坐标 
    double y = target_in_robot.pose.position.y;   // 横向偏移 
    double x = target_in_robot.pose.position.x;   // 前向距离 
 
    // 计算距离平方（避免开方运算）
    double L_sq = x*x + y*y;
 
    // 防止除以零（当目标点与机器人重合时）
    const double MIN_DIST_SQ = 0.0001;  // 10cm^2 
    if (L_sq < MIN_DIST_SQ) {
        return 0;
    }
    float alpha = atan2(y, x);
    double angular_z =  atan(2.0 * CARL_ *  sin(alpha) / sqrt(L_sq)) * 57.3; //单位角度

    // 计算角速度：0.15 为 单侧轮距（轮轴中心到驱动轮的纵向距离）
    // double angular_z =  atan(2.0 * CARL_ * y / L_sq) * 57.3; //单位角度
    // ROS_INFO("angular_z: %.2f,L_sq: %.2f,y: %.2f,current_vel: %.2f",angular_z,L_sq,y,current_velocity_);
    // 速度过低时禁止转向（防止零速时的抖动）
    // const double MIN_VELOCITY = 0.05;  // 5cm/s 
    // if (fabs(filtered_vel) < MIN_VELOCITY) {
    //     return 0;
    // }
    // return angular_z;
    // 接近终点时的位姿校正增强（仅在前进模式下）
    if (!is_reverse_mode_) {
        geometry_msgs::PoseStamped robot_pose;
        if (costmap_ros_->getRobotPose(robot_pose)) {
            double dx = global_plan_.back().pose.position.x - robot_pose.pose.position.x;
            double dy = global_plan_.back().pose.position.y - robot_pose.pose.position.y;
            double dist_to_goal = hypot(dx, dy);
            
            // 在接近终点时增强位姿校正
            const double pose_adjust_distance = 0.25;  // 位姿校正距离
            if (dist_to_goal < pose_adjust_distance) {
                // 计算目标朝向与当前朝向的偏差
                double goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);
                double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
                double angle_error = angles::shortest_angular_distance(robot_yaw, goal_yaw);
                
                // 直接基于角度偏差计算校正角速度
                const double kp_orientation = 1.5;  // 位姿校正增益
                double orientation_correction = kp_orientation * angle_error * 57.3;  // 转换为度
                
                // 将位姿校正与纯追踪结果结合
                double correction_weight = std::max(0.3, 1.0 - dist_to_goal / pose_adjust_distance);
                angular_z = angular_z * (1.0 - correction_weight) + orientation_correction * correction_weight;
            }
        }
    }
    
    // 角速度限幅 
    return (angular_z < -max_angular_) ? -max_angular_ :
          (angular_z > max_angular_) ? max_angular_ :
          angular_z;
}
 
/**
 * @brief 计算线速度
 * @param angular_z 计算出的角度 
 */
double PurePursuitPlanner::acalculateVel(geometry_msgs::PoseStamped target_pose, double angular_z) {
    // 根据模式选择最大速度
    double max_vel = is_reverse_mode_ ? max_reverse_velocity_ : max_linear_velocity_;
    double current_vel = fabs(current_velocity_ );

    //目标速度按照最大速度
    double target_linear_vel = max_vel;
    
    // 转弯减速功能：根据转向角度动态调整速度
    double abs_angular = fabs(angular_z);
    if (abs_angular > decel_angle_threshold_) {
        // 计算转向角度占比 [0,1]
        double angle_ratio = std::min(abs_angular / max_angular_, 1.0);
        
        // 应用转弯减速公式：角度越大，速度越小
        // 使用平方函数使减速更加平滑
        double turn_speed_factor = 1.0 - (angle_ratio * angle_ratio * decel_ratio_);
        turn_speed_factor = std::max(turn_speed_factor, 0.2); // 最小保持20%的速度
        
        target_linear_vel = max_vel * turn_speed_factor;
        
        // 确保转弯时速度不超过最大角速度对应的线速度
        target_linear_vel = std::min(target_linear_vel, max_angular_vel_);
        
        ROS_DEBUG("Turn deceleration: angle=%.2f°, factor=%.2f, target_vel=%.2f", 
                  abs_angular, turn_speed_factor, target_linear_vel);
    }

    //接近终点时减速
    // if(arePosesEqual(global_plan_.back(),target_pose)){
    //     if(current_vel > 0.13){
    //         target_linear_vel  = current_vel * 0.8;
    //         return target_linear_vel;
    //     }else{
    //         return current_vel > 0.08 ? current_vel : 0.08;
    //     }
    // }
    // 接近终点时的线性减速逻辑（仅在前进模式下生效）
    if (!is_reverse_mode_) {
        geometry_msgs::PoseStamped robot_pose;
        if (costmap_ros_->getRobotPose(robot_pose)) {
            // 计算到终点的距离
            double dx = global_plan_.back().pose.position.x - robot_pose.pose.position.x;
            double dy = global_plan_.back().pose.position.y - robot_pose.pose.position.y;
            double dist_to_goal = hypot(dx, dy);
            
            // 减速区间设置：终点前0.3米开始线性减速
            const double decel_distance = 0.3;
            const double min_speed = 0.05;  // 最小速度，避免完全停止
            
            if (dist_to_goal < decel_distance) {
                // 线性减速：速度与距离成正比
                // 当距离为0.3米时，速度为最大速度
                // 当距离为0米时，速度为最小速度
                double speed_ratio = dist_to_goal / decel_distance;
                target_linear_vel = min_speed + (max_vel - min_speed) * speed_ratio;
                
                // 位姿校正：在接近终点时根据角度偏差进一步调整速度
                double goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);
                double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
                double angle_diff = fabs(angles::shortest_angular_distance(robot_yaw, goal_yaw));
                
                // 当角度偏差较大时，进一步降低速度以便精确调整位姿
                const double angle_threshold = 0.2;  // 约11.5度
                if (angle_diff > angle_threshold) {
                    double angle_factor = std::max(0.3, 1.0 - (angle_diff - angle_threshold) / (M_PI - angle_threshold));
                    target_linear_vel *= angle_factor;
                }
                
                // 确保速度不低于最小值
                target_linear_vel = std::max(target_linear_vel, min_speed);
                
                return target_linear_vel;
            }
        }
    }

    // 速度差过大时限制加速度 (要减速，不可加速 所以要求target_linear_vel>current_velocity_ + decel_velocity_threshold_)
    if ((target_linear_vel - current_vel) > decel_velocity_threshold_ ) {
        target_linear_vel  = current_vel + decel_velocity_threshold_;
    }

    //限制最大速度不能超过阈值
    if (target_linear_vel > max_vel)
    {
        target_linear_vel = max_vel;
    }
    // ROS_INFO("target_linear_vel:%.2f",  decel_velocity_threshold_);

    return target_linear_vel;
}
 
/**
 * @brief 发布局部路径（用于可视化）
 * @param target_pose 目标点位姿 
 */
void PurePursuitPlanner::publishLocalPlan(const geometry_msgs::PoseStamped& target_pose) {
    nav_msgs::Path local_path;
    local_path.header.frame_id  = global_plan_.front().header.frame_id; 
    local_path.header.stamp  = ros::Time::now();
 
    // 获取当前机器人位姿 
    geometry_msgs::PoseStamped robot_pose;
    if (costmap_ros_->getRobotPose(robot_pose)) {
        // 构建局部路径：当前位姿 -> 目标点 
        local_path.poses.push_back(robot_pose); 
        local_path.poses.push_back(target_pose); 
        
        // 发布可视化消息 
        local_plan_pub_.publish(local_path);
        target_pose_pub_.publish(target_pose);
    }
}

} // namespace pure_pursuit_planner 
 
// 插件注册宏 
PLUGINLIB_EXPORT_CLASS(pure_pursuit_planner::PurePursuitPlanner, nav_core::BaseLocalPlanner)