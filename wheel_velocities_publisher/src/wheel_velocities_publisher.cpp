#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node
{
public:
    WheelVelocitiesPublisher() : Node("wheel_velocities_publisher"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Wheel Velocities Publisher Node has been initialized.");
        
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
        timer_ = this->create_wall_timer(
            3s, std::bind(&WheelVelocitiesPublisher::publish_wheel_speeds, this));
    }

private:
    void publish_wheel_speeds()
    {
        auto message = std_msgs::msg::Float32MultiArray();
        std::vector<float> wheel_speeds(4, 0.0);

        switch (count_)
        {
        case 0:
            RCLCPP_INFO(this->get_logger(), "Move forward");
            wheel_speeds = {1.0, 1.0, 1.0, 1.0};
            break;
        case 1:
            RCLCPP_INFO(this->get_logger(), "Move backward");
            wheel_speeds = {-1.0, -1.0, -1.0, -1.0};
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "Move left");
            wheel_speeds = {-1.0, 1.0, -1.0, 1.0};
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "Move right");
            wheel_speeds = {1.0, -1.0, 1.0, -1.0};
            break;
        case 4:
            RCLCPP_INFO(this->get_logger(), "Turn clockwise");
            wheel_speeds = {1.0, -1.0, -1.0, 1.0};
            break;
        case 5:
            RCLCPP_INFO(this->get_logger(), "Turn counter-clockwise");
            wheel_speeds = {-1.0, 1.0, 1.0, -1.0};
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Stop");
            wheel_speeds = {0.0, 0.0, 0.0, 0.0};
            break;
        }

        message.data = wheel_speeds;
        publisher_->publish(message);

        if (count_ < 6)
        {
            count_++;
        }
        else
        {
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
    rclcpp::shutdown();
    return 0;
}