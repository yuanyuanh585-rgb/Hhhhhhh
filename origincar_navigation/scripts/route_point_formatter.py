#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node


class RoutePointFormatter(Node):
    def __init__(self):
        super().__init__('route_point_formatter')

        self.topic = self.declare_parameter('topic', '/initialpose').value
        self.once = bool(self.declare_parameter('once', False).value)
        self.include_yaw_deg = bool(self.declare_parameter('include_yaw_deg', True).value)
        self.precision = max(1, int(self.declare_parameter('precision', 6).value))

        self.pose_stamped_sub = None
        self.pose_with_cov_sub = None
        if self.topic == '/goal_pose':
            self.pose_stamped_sub = self.create_subscription(
                PoseStamped, self.topic, self._on_pose_stamped, 10)
        elif self.topic == '/initialpose':
            self.pose_with_cov_sub = self.create_subscription(
                PoseWithCovarianceStamped, self.topic, self._on_pose_with_covariance, 10)
        else:
            self.pose_stamped_sub = self.create_subscription(
                PoseStamped, self.topic, self._on_pose_stamped, 10)
            self.pose_with_cov_sub = self.create_subscription(
                PoseWithCovarianceStamped, self.topic, self._on_pose_with_covariance, 10)

        self.get_logger().info(
            f'Listening on {self.topic}. Click a pose in RViz to print a global_routes.yaml point.')

    def _format(self, value: float) -> str:
        return f'{value:.{self.precision}f}'

    def _yaw_deg(self, orientation) -> float:
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def _print_pose(self, pose, source: str):
        orientation = pose.orientation
        print('')
        print(f'# from {source}')
        print(f'- x: {self._format(pose.position.x)}')
        print(f'  y: {self._format(pose.position.y)}')
        print(
            '  orientation: '
            f'{{x: {self._format(orientation.x)}, '
            f'y: {self._format(orientation.y)}, '
            f'z: {self._format(orientation.z)}, '
            f'w: {self._format(orientation.w)}}}'
        )
        if self.include_yaw_deg:
            print(f'  # yaw_deg: {self._format(self._yaw_deg(orientation))}')
        print('', flush=True)

        if self.once:
            rclpy.shutdown()

    def _on_pose_stamped(self, msg: PoseStamped):
        self._print_pose(msg.pose, self.topic)

    def _on_pose_with_covariance(self, msg: PoseWithCovarianceStamped):
        self._print_pose(msg.pose.pose, self.topic)


def main():
    rclpy.init()
    node = RoutePointFormatter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
