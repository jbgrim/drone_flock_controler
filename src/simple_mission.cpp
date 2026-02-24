/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <drone_flock_controler_interfaces/msg/drone_global_position.hpp>
#include <drone_flock_controler_interfaces/msg/target_position.hpp>

#include <chrono>
#include <iostream>
#include <drone_flock_controler/Vec3.hpp>

using namespace drone_flock_controler_interfaces::msg;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class SimpleMission : public rclcpp::Node
{
public:
	SimpleMission() : Node("offboard_control")
	{
        // declaring parameters
        this->declare_parameter<int>("instance_id", 1);
		this->declare_parameter<double>("k_mig", 1.0);
		this->declare_parameter<double>("k_sep", 1.0);
		this->declare_parameter<double>("k_coh", 1.0);
        instance_id_ = this->get_parameter("instance_id").as_int();
		k_mig_ = this->get_parameter("k_mig").as_double();
		k_sep_ = this->get_parameter("k_sep").as_double();
		k_coh_ = this->get_parameter("k_coh").as_double();

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_" + std::to_string(instance_id_) + "/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_" + std::to_string(instance_id_) + "/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_" + std::to_string(instance_id_) + "/fmu/in/vehicle_command", 10);
		drone_position_publisher_ = this->create_publisher<DroneGlobalPosition>("/drone_flock_controler/positions", 10);

		vehicle_status_subscription_ = this->create_subscription<VehicleStatus>(
			"/px4_" + std::to_string(instance_id_) + "/fmu/out/vehicle_status_v1",
			rclcpp::SensorDataQoS(),
			[this](const VehicleStatus::SharedPtr msg) {
				if (this->target_system_ == 0) {
					this->target_system_ = msg->system_id;
					RCLCPP_INFO(this->get_logger(), "Set target system to %u", this->target_system_);
				}
				this->arming_state_ = msg->arming_state;
			}
		);

		vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
			"/px4_" + std::to_string(instance_id_) + "/fmu/out/vehicle_local_position_v1",
			rclcpp::SensorDataQoS(),
			[this](const VehicleLocalPosition::SharedPtr msg) {
				this->local_position_.x = msg->x;
				this->local_position_.y = msg->y;
				this->local_position_.z = msg->z;
				if (this->instance_id_ & 0b1) {
					this->local_position_.x += 2.0f;
				}
				if (this->instance_id_ & 0b10) {
					this->local_position_.y += 2.0f;
				}
				if (this->instance_id_ & 0b100) {
					this->local_position_.x += 4.0f;
				}
				if (this->instance_id_ & 0b1000) {
					this->local_position_.y += 4.0f;
				}
				DroneGlobalPosition position_msg;
				position_msg.instance_id = this->instance_id_;
				position_msg.x = this->local_position_.x;
				position_msg.y = this->local_position_.y;
				position_msg.z = this->local_position_.z;
				this->drone_position_publisher_->publish(position_msg);
			}
		);

		vehicle_global_position_subscription_ = this->create_subscription<VehicleGlobalPosition>(
			"/px4_" + std::to_string(instance_id_) + "/fmu/out/vehicle_global_position",
			rclcpp::SensorDataQoS(),
			[this](const VehicleGlobalPosition::SharedPtr msg) {
				this->global_position_ = *msg;
			}
		);

		drone_positions_subscription_ = this->create_subscription<DroneGlobalPosition>(
			"/drone_flock_controler/positions",
			rclcpp::SensorDataQoS(),
			[this](const DroneGlobalPosition::SharedPtr msg) {
				// Update the position of the other drone in the flock
				bool found = false;
				for (auto& pos : this->drone_positions_) {
					if (pos.instance_id == msg->instance_id) {
						pos.x = msg->x;
						pos.y = msg->y;
						pos.z = msg->z;
						found = true;
						break;
					}
				}
				if (!found) {
					this->drone_positions_.push_back(*msg);
				}
			}
		);

		target_position_subscription_ = this->create_subscription<TargetPosition>(
			"/drone_flock_controler/target_position",
			rclcpp::SensorDataQoS(),
			[this](const TargetPosition::SharedPtr msg) {
				this->target_position_ = *msg;
			}
		);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			if (target_system_ == 0) {
				// System not connected yet
				return;
			}

			if (offboard_setpoint_counter_ >= 10 && offboard_setpoint_counter_ < 31 && offboard_setpoint_counter_ % 5 == 0 && arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 31
			if (offboard_setpoint_counter_ < 31) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<DroneGlobalPosition>::SharedPtr drone_position_publisher_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_position_subscription_;
	rclcpp::Subscription<DroneGlobalPosition>::SharedPtr drone_positions_subscription_;
	rclcpp::Subscription<TargetPosition>::SharedPtr target_position_subscription_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    
    int instance_id_;
    uint32_t target_system_ = 0;
	uint8_t arming_state_ = 0;
	Vec3 local_position_;
	TargetPosition target_position_;
	VehicleGlobalPosition global_position_;
	std::vector<DroneGlobalPosition> drone_positions_;
	float k_mig_;
	float k_sep_;
	float k_coh_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	Vec3 compute_flocking_velocity(const DroneGlobalPosition& other_drone);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void SimpleMission::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void SimpleMission::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void SimpleMission::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void SimpleMission::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {std::nanf(""), std::nanf(""), target_position_.z}; // ignore position setpoint but force altitude to 5m
	msg.acceleration = {std::nanf(""), std::nanf(""), std::nanf("")}; // ignore acceleration control loop feeding

	Vec3 p_mig(target_position_.x, target_position_.y, target_position_.z);
	Vec3 r_mig = p_mig - local_position_;
	float distance = r_mig.magnitude();
	if (distance < 0.1f) { // goal reached don't move
		msg.velocity = {0.f, 0.f, 0.f};
	} else {
		msg.velocity = ((k_mig_/distance) * r_mig).to_array();
	}
	// add flocking velocity from other drones
	size_t num_drones = drone_positions_.size() - 1;
	for (const auto& other_drone : drone_positions_) {
		Vec3 v_flock = compute_flocking_velocity(other_drone);
		msg.velocity[0] += v_flock.x / num_drones;
		msg.velocity[1] += v_flock.y / num_drones;
		msg.velocity[2] += v_flock.z / num_drones;
	}

	msg.yaw = target_position_.yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void SimpleMission::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = target_system_;
	msg.target_component = 1;
	msg.source_system = target_system_;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

Vec3 SimpleMission::compute_flocking_velocity(const DroneGlobalPosition& other_drone)
{
	if (other_drone.instance_id == this->instance_id_) {
		return Vec3(0.f, 0.f, 0.f); // ignore self
	}

	// Simple flocking behavior: move towards the other drone if it's too far, otherwise stay still
	Vec3 other_position(other_drone.x, other_drone.y, other_drone.z);
	Vec3 r = other_position - local_position_;
	float distance = r.magnitude();

	Vec3 v_sep = r * (-k_sep_ / (distance * distance)); // separation velocity to avoid collisions

	Vec3 v_coh = r * k_coh_; // cohesion velocity to move towards the other drone

	return v_sep + v_coh;
}


int main(int argc, char *argv[])
{
	/*std::cout << "Connecting to PX4..." << std::endl;
	// Initialize MAVSDK with GroundStation component type
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};

    // Add connection
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
   if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    // Wait for the system to connect
    std::cout << "Waiting for system to connect..." << std::endl;
    while (mavsdk.systems().size() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Get the connected system
    auto system = mavsdk.systems().at(0);
   if (!system->is_connected()) {
        std::cerr << "System not connected" << std::endl;
        return 1;
    }
	std::cout << "Starting offboard control node..." << std::endl;*/
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimpleMission>());

	rclcpp::shutdown();
	return 0;
}
