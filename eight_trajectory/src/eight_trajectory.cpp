#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <Eigen/Dense>

class AbsoluteMotionNode : public rclcpp::Node {
public:
    AbsoluteMotionNode() : Node("absolute_motion"), phi_(0.0) {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AbsoluteMotionNode::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Node initialized");
        rclcpp::sleep_for(std::chrono::seconds(1));
        executeMotions();
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, phi_);
        position_ = msg->pose.pose.position;

        // Debugging information to ensure the callback is being called
        RCLCPP_INFO(this->get_logger(), "Odometry callback: x = %f, y = %f, z = %f, phi = %f", position_.x, position_.y, position_.z, phi_);
    }

    std::tuple<double, double, double> velocity2twist(double dphi, double dx, double dy) {
        // Using differential changes in phi, x, and y to compute the twist
        Eigen::Vector3d v(dphi, dx, dy);
        return std::make_tuple(v(0), v(1), v(2));
    }

    std::vector<float> twist2wheels(double wz, double vx, double vy) {
        double l = 0.500 / 2;
        double r = 0.254 / 2;
        double w = 0.548 / 2;

        Eigen::MatrixXd H(4, 3);
        H << -l-w, 1, -1,
              l+w, 1,  1,
              l+w, 1, -1,
             -l-w, 1,  1;
        H /= r;

        Eigen::Vector3d twist(wz, vx, vy);
        Eigen::VectorXd u = H * twist;
        std::vector<float> result(u.data(), u.data() + u.size());
        return result;
    }

    void executeMotions() {
        std::vector<std::tuple<double, double, double>> waypoints = {
            {0.0, 1, -1}, 
            {0.0, 1, 1}, 
            {0.0, 1, 1}, 
            {1.5708, 1, -1}, 
            {-3.1415, -1, -1}, 
            {0.0, -1, 1}, 
            {0.0, -1, 1}, 
            {0.0, -1, -1}
        };

        for (const auto& waypoint : waypoints) {
            double dphi, dx, dy;
            std::tie(dphi, dx, dy) = waypoint;
            RCLCPP_INFO(this->get_logger(), "Changing to new waypoint: dphi = %f, dx = %f, dy = %f", dphi, dx, dy);
            RCLCPP_INFO(this->get_logger(), "Current position: x = %f, y = %f, z = %f", position_.x, position_.y, position_.z);
            int iterations = 300;  // Number of iterations for each waypoint, adjust as needed

            for (int i = 0; i < iterations; ++i) {
                double wz, vx, vy;
                std::tie(wz, vx, vy) = velocity2twist(dphi, dx, dy);
                std::vector<float> u = twist2wheels(wz, vx, vy);

                std_msgs::msg::Float32MultiArray msg;
                msg.data = u;
                pub_->publish(msg);
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }

        std_msgs::msg::Float32MultiArray stop_msg;
        stop_msg.data = {0, 0, 0, 0};
        pub_->publish(stop_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    double phi_;
    geometry_msgs::msg::Point position_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AbsoluteMotionNode>());
    rclcpp::shutdown();
    return 0;
}
