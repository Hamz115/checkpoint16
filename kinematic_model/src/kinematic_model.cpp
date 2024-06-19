#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <functional>
#include <string>

using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Twist = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;
using namespace std::placeholders;

class KinematicModel : public rclcpp::Node
{
public:
    KinematicModel() : Node("kinematics_model_publisher") {
        RCLCPP_INFO(this->get_logger(), "Initialized kinematics model publisher node.");
        publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<Float32MultiArray>("/wheel_speed", 10, std::bind(&KinematicModel::topic_callback, this, _1));     
    }

private:
    // General attributes
    rclcpp::Publisher<Twist>::SharedPtr publisher_;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr subscription_;

    // Robot physical properties
    float w = 0.134845;
    float r = 0.05;
    float l = 0.085;

    void topic_callback(const Float32MultiArray::SharedPtr msg) {
        Eigen::MatrixXd HolonomicMatrix(4, 3);
        HolonomicMatrix << -l-w, 1, -1,
                            l+w, 1,  1,
                            l+w, 1, -1,
                           -l-w, 1,  1;

        Eigen::MatrixXd U(4, 1);
        U << msg->data[0],
             msg->data[1],
             msg->data[2],
             msg->data[3];

        Eigen::MatrixXd CommandVels(3, 1);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(HolonomicMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd pinvHolonomicMatrix = svd.solve(Eigen::MatrixXd::Identity(HolonomicMatrix.rows(), HolonomicMatrix.rows()));

        CommandVels = pinvHolonomicMatrix * r * U;

        Twist vel_msg;
        vel_msg.angular.z = float(CommandVels(0, 0));
        vel_msg.linear.x = float(CommandVels(1, 0));
        vel_msg.linear.y = float(CommandVels(2, 0));

        RCLCPP_DEBUG(this->get_logger(), "Received velocities:");
        RCLCPP_DEBUG(this->get_logger(), "Angular Z: %.2f", vel_msg.angular.z);
        RCLCPP_DEBUG(this->get_logger(), " Linear X: %.2f", vel_msg.linear.x);
        RCLCPP_DEBUG(this->get_logger(), " Linear Y: %.2f", vel_msg.linear.y);   

        publisher_->publish(vel_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicModel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}