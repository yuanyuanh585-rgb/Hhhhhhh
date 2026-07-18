# Origincar Simulation Plan

## Goal

Create a Gazebo Fortress simulation directly under `/home/xjl/digua` for validating ROS2 navigation algorithms on an Ackermann Origincar model.

## Steps

1. Create `doc` and `src/origincar_sim`.
2. Convert `智能车地图源文件.avif` into a Gazebo-readable PNG texture.
3. Build a 5m x 5m arena model with 0.5m walls.
4. Create `origincar_ackermann.urdf.xacro` based on the original Origincar dimensions and visual style.
5. Add LiDAR, IMU, odometry, and TF outputs.
6. Implement a Gazebo Fortress kinematic Ackermann plugin.
7. Add launch files for Gazebo simulation and RViz model inspection.
8. Verify xacro expansion, build, launch file discovery, texture existence, and generated directory layout.

## Acceptance Checks

- New files are under `doc` and `src/origincar_sim`.
- The original `origincar_description` package remains unchanged.
- `smart_car_map.png` exists in the arena model texture directory.
- `xacro` expands `origincar_ackermann.urdf.xacro` without XML errors.
- `colcon build --packages-select origincar_sim` succeeds after Gazebo Fortress and ros_gz dependencies are installed.
- Gazebo publishes `/scan`, `/imu/data`, `/odom`, and `/tf`.
