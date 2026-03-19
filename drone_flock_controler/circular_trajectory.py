#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from drone_flock_controler_interfaces.msg import TargetPosition


class CircularTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("circular_trajectory_publisher")

        self.declare_parameter("radius", 5.0)
        self.declare_parameter("height", -5.0)
        self.declare_parameter("angular_velocity", 0.5)

        self.radius = float(self.get_parameter("radius").value)
        self.height = float(self.get_parameter("height").value)
        self.angular_velocity = float(self.get_parameter("angular_velocity").value)

        self.target_position_publisher = self.create_publisher(
            TargetPosition,
            "/drone_flock_controler/target_position",
            10,
        )

        self.start_time_ns = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.1, self._on_timer)

        self.get_logger().info("Circular trajectory node started")

    def _on_timer(self) -> None:
        elapsed_time = (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9
        angle = self.angular_velocity * elapsed_time

        msg = TargetPosition()
        msg.x = float(self.radius * math.cos(angle))
        msg.y = float(self.radius * math.sin(angle))
        msg.z = float(self.height)
        msg.yaw = float(angle)

        self.target_position_publisher.publish(msg)

        self.get_logger().debug(
            f"Publishing target position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, yaw={msg.yaw:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CircularTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
