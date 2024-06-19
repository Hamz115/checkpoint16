#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <string>

using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using namespace std::chrono_literals;

class WheelVelocities : public rclcpp::Node
{
public:
    WheelVelocities() : Node("wheel_velocities_publisher") {
        RCLCPP_INFO(this->get_logger(), "Initialized wheel velocities publisher node.");
        publisher_ = this->create_publisher<Float32MultiArray>("/wheel_speed", 10);
        this->declare_parameter<float>("linear_x", 0.5);
        this->declare_parameter<float>("linear_y", 0.5);
        this->declare_parameter<float>("angular_z", 0.5);
        this->get_parameter("linear_x", linear_x);
        this->get_parameter("linear_y", linear_y);
        this->get_parameter("angular_z", angular_z);
        this->publish_velocities();
    }

private:
    // General attributes
    rclcpp::Publisher<Float32MultiArray>::SharedPtr publisher_;
    
    // Robot physical properties
    float w = 0.134845;
    float r = 0.05;
    float l = 0.085;

    // Velocity commands
    float linear_x;
    float linear_y;
    float angular_z;

    void publish_velocities() {
        Eigen::MatrixXd CommandVels(3, 1);
        // Start motion
        rclcpp::sleep_for(3s);
        // Move forward
        CommandVels << 0, linear_x, 0;
        send_message(CommandVels, "Move forward");
        rclcpp::sleep_for(3s);
        // Move backward
        CommandVels << 0, -linear_x, 0;
        send_message(CommandVels, "Move backward");
        rclcpp::sleep_for(3s);
        // Move left
        CommandVels << 0, 0, linear_y;
        send_message(CommandVels, "Move left");
        rclcpp::sleep_for(3s);
        // Move right
        CommandVels << 0, 0, -linear_y;
        send_message(CommandVels, "Move right");
        rclcpp::sleep_for(3s);
        // Move clockwise
        CommandVels << angular_z, 0, 0;
        send_message(CommandVels, "Turn clockwise");
        rclcpp::sleep_for(3s);
        // Move counter-clockwise
        CommandVels << -angular_z, 0, 0;
        send_message(CommandVels, "Turn counter-clockwise");
        rclcpp::sleep_for(3s);
        // Stop motion
        CommandVels << 0, 0, 0;
        send_message(CommandVels, "Stop");
        rclcpp::sleep_for(3s);
    }

    void send_message(Eigen::MatrixXd CommandVels, std::string Message) {
        Eigen::MatrixXd HolonomicMatrix(4, 3);
        HolonomicMatrix << -l-w, 1, -1,
                            l+w, 1,  1,
                            l+w, 1, -1,
                           -l-w, 1,  1;


        Eigen::MatrixXd WheelsVel(4, 1);
        WheelsVel = (1/r) * HolonomicMatrix * CommandVels;

        Float32MultiArray vel_message;
        vel_message.data = {float(WheelsVel(0, 0)),
                            float(WheelsVel(1, 0)), 
                            float(WheelsVel(2, 0)), 
                            float(WheelsVel(3, 0))};

        RCLCPP_INFO(this->get_logger(), "%s.", Message.c_str());
        publisher_->publish(vel_message);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelVelocities>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}