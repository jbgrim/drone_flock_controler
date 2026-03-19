#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

from drone_flock_controler_interfaces.msg import DroneGlobalPosition
from drone_flock_controler_interfaces.msg import TargetPosition


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]


class SimpleMission(Node):
    def __init__(self) -> None:
        super().__init__("offboard_control")

        self.declare_parameter("instance_id", 1)
        self.declare_parameter("k_mig", 1.0)
        self.declare_parameter("k_sep", 1.0)
        self.declare_parameter("k_coh", 1.0)

        self.instance_id = int(self.get_parameter("instance_id").value)
        self.k_mig = float(self.get_parameter("k_mig").value)
        self.k_sep = float(self.get_parameter("k_sep").value)
        self.k_coh = float(self.get_parameter("k_coh").value)

        prefix = f"/px4_{self.instance_id}/fmu"

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            f"{prefix}/in/offboard_control_mode",
            10,
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            f"{prefix}/in/trajectory_setpoint",
            10,
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            f"{prefix}/in/vehicle_command",
            10,
        )
        self.drone_position_publisher = self.create_publisher(
            DroneGlobalPosition,
            "/drone_flock_controler/positions",
            10,
        )

        self.target_system = 0
        self.arming_state = 0
        self.local_position = Vec3()
        self.target_position = TargetPosition()
        self.global_position = VehicleGlobalPosition()
        self.drone_positions: Dict[int, DroneGlobalPosition] = {}
        self.offboard_setpoint_counter = 0

        self.create_subscription(
            VehicleStatus,
            f"{prefix}/out/vehicle_status_v1",
            self._on_vehicle_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f"{prefix}/out/vehicle_local_position_v1",
            self._on_vehicle_local_position,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            f"{prefix}/out/vehicle_global_position",
            self._on_vehicle_global_position,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DroneGlobalPosition,
            "/drone_flock_controler/positions",
            self._on_drone_position,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TargetPosition,
            "/drone_flock_controler/target_position",
            self._on_target_position,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(0.1, self._on_timer)

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        if self.target_system == 0:
            self.target_system = msg.system_id
            self.get_logger().info(f"Set target system to {self.target_system}")
        self.arming_state = msg.arming_state

    def _on_vehicle_local_position(self, msg: VehicleLocalPosition) -> None:
        x = float(msg.x)
        y = float(msg.y)

        if self.instance_id & 0b1:
            x += 2.0
        if self.instance_id & 0b10:
            y += 2.0
        if self.instance_id & 0b100:
            x += 4.0
        if self.instance_id & 0b1000:
            y += 4.0

        self.local_position = Vec3(x=x, y=y, z=float(msg.z))

        position_msg = DroneGlobalPosition()
        position_msg.instance_id = self.instance_id
        position_msg.x = self.local_position.x
        position_msg.y = self.local_position.y
        position_msg.z = self.local_position.z
        self.drone_position_publisher.publish(position_msg)

    def _on_vehicle_global_position(self, msg: VehicleGlobalPosition) -> None:
        self.global_position = msg

    def _on_drone_position(self, msg: DroneGlobalPosition) -> None:
        self.drone_positions[int(msg.instance_id)] = msg

    def _on_target_position(self, msg: TargetPosition) -> None:
        self.target_position = msg

    def _on_timer(self) -> None:
        if self.target_system == 0:
            return

        if (
            self.offboard_setpoint_counter >= 10
            and self.offboard_setpoint_counter < 31
            and self.offboard_setpoint_counter % 5 == 0
            and self.arming_state != VehicleStatus.ARMING_STATE_ARMED
        ):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
            )
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter < 31:
            self.offboard_setpoint_counter += 1

    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
        )
        self.get_logger().info("Arm command send")

    def disarm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0
        )
        self.get_logger().info("Disarm command send")

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, float(self.target_position.z)]
        msg.acceleration = [math.nan, math.nan, math.nan]

        migration_target = Vec3(
            self.target_position.x, self.target_position.y, self.target_position.z
        )
        migration_vector = migration_target - self.local_position
        distance = migration_vector.magnitude()

        if distance < 0.1:
            msg.velocity = [0.0, 0.0, 0.0]
        else:
            msg.velocity = (migration_vector * (self.k_mig / distance)).to_list()

        neighbors = [
            pos
            for drone_id, pos in self.drone_positions.items()
            if drone_id != self.instance_id
        ]
        if neighbors:
            inv_neighbors = 1.0 / float(len(neighbors))
            for other_drone in neighbors:
                flock_velocity = self.compute_flocking_velocity(other_drone)
                msg.velocity[0] += flock_velocity.x * inv_neighbors
                msg.velocity[1] += flock_velocity.y * inv_neighbors
                msg.velocity[2] += flock_velocity.z * inv_neighbors

        msg.yaw = float(self.target_position.yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(
        self, command: int, param1: float = 0.0, param2: float = 0.0
    ) -> None:
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = int(self.target_system)
        msg.target_component = 1
        msg.source_system = int(self.target_system)
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(msg)

    def compute_flocking_velocity(self, other_drone: DroneGlobalPosition) -> Vec3:
        if int(other_drone.instance_id) == self.instance_id:
            return Vec3()

        other_position = Vec3(
            float(other_drone.x), float(other_drone.y), float(other_drone.z)
        )
        rel = other_position - self.local_position
        distance = rel.magnitude()

        if distance < 1e-3:
            return Vec3()

        separation = rel * (-self.k_sep / (distance * distance))
        cohesion = rel * self.k_coh
        return separation + cohesion


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
