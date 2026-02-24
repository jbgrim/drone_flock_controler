#include <rclcpp/rclcpp.hpp>
#include <drone_flock_controler_interfaces/msg/target_position.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using namespace drone_flock_controler_interfaces::msg;

class CircularTrajectoryNode : public rclcpp::Node
{
public:
    CircularTrajectoryNode() : Node("circular_trajectory_publisher")
    {
        // Declare parameters
        this->declare_parameter<float>("radius", 5.0f);
        this->declare_parameter<float>("height", -5.0f);
        this->declare_parameter<float>("angular_velocity", 0.5f);

        radius_ = this->get_parameter("radius").as_double();
        height_ = this->get_parameter("height").as_double();
        angular_velocity_ = this->get_parameter("angular_velocity").as_double();

        target_position_publisher_ = this->create_publisher<TargetPosition>("/drone_flock_controler/target_position", 10);

        auto timer_callback = [this]() -> void {
            // Calculate angle based on elapsed time
            auto now = this->get_clock()->now();
            double elapsed_time = (now.nanoseconds() - start_time_.nanoseconds()) / 1e9;
            double angle = angular_velocity_ * elapsed_time;

            // Calculate circular trajectory position
            TargetPosition msg{};
            msg.x = radius_ * std::cos(angle);
            msg.y = radius_ * std::sin(angle);
            msg.z = height_;
            msg.yaw = static_cast<float>(angle); // Yaw points along the tangent

            target_position_publisher_->publish(msg);

            RCLCPP_DEBUG(this->get_logger(), 
                "Publishing target position: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                msg.x, msg.y, msg.z, msg.yaw);
        };

        start_time_ = this->get_clock()->now();
        timer_ = this->create_wall_timer(100ms, timer_callback);

        RCLCPP_INFO(this->get_logger(), "Circular trajectory node started");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TargetPosition>::SharedPtr target_position_publisher_;

    float radius_;
    float height_;
    float angular_velocity_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularTrajectoryNode>());

    rclcpp::shutdown();
    return 0;
}