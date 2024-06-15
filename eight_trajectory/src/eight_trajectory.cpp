#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class EightTrajectory : public rclcpp::Node
{
public:
    EightTrajectory() : Node("eight_trajectory"), current_waypoint_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Eight Trajectory Node has been initialized.");
        
        wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&EightTrajectory::odom_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&EightTrajectory::control_loop, this));

        // Initialize waypoints
        waypoints_ = {
            {0.0, 1, -1},    // w1
            {0.0, 1, 1},     // w2
            {0.0, 1, 1},     // w3
            {1.5708, 1, -1}, // w4
            {-3.1415, -1, -1},// w5
            {0.0, -1, 1},    // w6
            {0.0, -1, 1},    // w7
            {0.0, -1, -1}    // w8
        };

        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update current position and orientation
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        current_yaw_ = tf2::impl::getYaw(q);
    }

    void control_loop()
    {
        if (current_waypoint_ < waypoints_.size())
        {
            auto wp = waypoints_[current_waypoint_];
            float dphi = wp[0];
            float dx = wp[1];
            float dy = wp[2];

            auto wheel_speeds = compute_wheel_speeds(dphi, dx, dy);
            publish_wheel_speeds(wheel_speeds);

            // Check if the waypoint is reached
            if (is_waypoint_reached(dx, dy, dphi))
            {
                current_waypoint_++;
            }
        }
        else
        {
            stop_robot();
        }
    }

    bool is_waypoint_reached(float dx, float dy, float dphi)
    {
        // Implement a simple check to determine if the waypoint is reached
        float goal_x = current_x_ + dx;
        float goal_y = current_y_ + dy;
        float goal_yaw = current_yaw_ + dphi;

        return (std::abs(current_x_ - goal_x) < 0.1 && std::abs(current_y_ - goal_y) < 0.1 && std::abs(current_yaw_ - goal_yaw) < 0.1);
    }

    std::vector<float> compute_wheel_speeds(float dphi, float dx, float dy)
    {
        // Implement the kinematic model to convert waypoint parameters to wheel speeds
        // For simplicity, assume a holonomic robot with direct mapping
        float v_x = dx;
        float v_y = dy;
        float omega_z = dphi;

        std::vector<float> wheel_speeds = {v_x - v_y - omega_z, v_x + v_y + omega_z, v_x + v_y - omega_z, v_x - v_y + omega_z};
        return wheel_speeds;
    }

    void publish_wheel_speeds(const std::vector<float>& wheel_speeds)
    {
        auto message = std_msgs::msg::Float32MultiArray();
        message.data = wheel_speeds;
        wheel_speed_publisher_->publish(message);
    }

    void stop_robot()
    {
        auto message = std_msgs::msg::Float32MultiArray();
        message.data = {0.0, 0.0, 0.0, 0.0};
        wheel_speed_publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    float current_x_;
    float current_y_;
    float current_yaw_;
    size_t current_waypoint_;

    std::vector<std::array<float, 3>> waypoints_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EightTrajectory>());
    rclcpp::shutdown();
    return 0;
}