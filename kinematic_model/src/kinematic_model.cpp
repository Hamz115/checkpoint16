#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

class KinematicModel : public rclcpp::Node
{
public:
    KinematicModel() : Node("kinematic_model")
    {
        RCLCPP_INFO(this->get_logger(), "Kinematic Model Node has been initialized.");
        wheel_speed_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_speed", 10, std::bind(&KinematicModel::wheel_speed_callback, this, std::placeholders::_1));
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void wheel_speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Wheel speed message does not contain 4 elements");
            return;
        }

        float v_rf = msg->data[0];  // right front wheel
        float v_lf = msg->data[1];  // left front wheel
        float v_rr = msg->data[2]; // right rear wheel
        float v_lr = msg->data[3]; // left rear wheel

        // Compute linear and angular velocities from wheel speeds for a holonomic robot
        float linear_x = (v_rf + v_lf + v_rr + v_lr) / 4.0;
        float linear_y = (-v_rf + v_lf - v_rr + v_lr) / 4.0;
        float angular_z = (-v_rf + v_lf + v_rr - v_lr) / 4.0;

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.linear.y = linear_y;
        twist_msg.angular.z = angular_z;

        twist_publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicModel>());
    rclcpp::shutdown();
    return 0;
}