#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <string>
#include <cmath>
#include <iostream>

using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Odometry = nav_msgs::msg::Odometry;
using namespace std::chrono_literals;
using namespace std::placeholders;

class EightTrajectory : public rclcpp::Node
{
public:
    EightTrajectory() : Node("eight_trajectory") {
        RCLCPP_INFO(this->get_logger(), "Initialized eight_trajectory node.");
        CommandVels << 0.0, 0.0, 0.0;
        CurrentOdom << 0.0, 0.0, 0.0;
        publisher_ = this->create_publisher<Float32MultiArray>("/wheel_speed", 10);
        subscription_ = this->create_subscription<Odometry>("/odometry/filtered", 10, std::bind(&EightTrajectory::topic_callback, this, _1));
        update_waypoint();
    }

private:
    // General attributes
    rclcpp::Publisher<Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr waypoint_timer_;
    rclcpp::TimerBase::SharedPtr velocities_timer_;

    // Robot physical properties
    float w = 0.134845;
    float r = 0.05;
    float l = 0.085;

    // Orietation and waypoints
    int current_waypoint = 0;

    Eigen::Vector3d CommandVels;
    Eigen::Vector3d CurrentOdom;
    Eigen::Vector3d OdomTarget;

    void update_waypoint() {
        RCLCPP_INFO(this->get_logger(), "Changing waypoint.");
        current_waypoint += 1;
        switch (current_waypoint) {
            case 1:
                CommandVels << 0.0, 1.0, -1.0;
                break;
            case 2:
                CommandVels << 0.0, 1.0, 1.0;
                break;
            case 3:
                CommandVels << 0.0, 1.0, 1.0;
                break;
            case 4:
                CommandVels << 1.5708, 1.0, -1.0;
                break;
            case 5:
                CommandVels << -3.1415, -1.0, -1.0;
                break;
            case 6:
                CommandVels << 0.0, -1.0, 1.0;
                break;
            case 7:
                CommandVels << 0.0, -1.0, 1.0;
                break;
            case 8:
                CommandVels << 0.0, -1.0, -1.0;
                break;
            default:
                rclcpp::shutdown();
        }
        OdomTarget = CurrentOdom + CommandVels;
    }

    void compute_velocities() {
        Eigen::MatrixXd RotationMatrix(3, 3);
        Eigen::MatrixXd TwistVels(3, 1);
        RotationMatrix << 1,    0,                        0,
                          0,    std::cos(CurrentOdom[0]), std::sin(CurrentOdom[0]),
                          0,   -std::sin(CurrentOdom[0]), std::cos(CurrentOdom[0]);
        TwistVels = RotationMatrix * (OdomTarget - CurrentOdom);
        float norm = float((OdomTarget - CurrentOdom).norm());
        send_message(TwistVels, "Update velocities");
        if (norm < 0.1) {update_waypoint();}
    }

    void send_message(Eigen::MatrixXd TwistVels, std::string Message) {
        Eigen::MatrixXd HolonomicMatrix(4, 3);
        HolonomicMatrix << -l-w, 1, -1,
                            l+w, 1,  1,
                            l+w, 1, -1,
                           -l-w, 1,  1;

        Eigen::MatrixXd WheelsVel(4, 1);
        WheelsVel = (1/r) * HolonomicMatrix * TwistVels;

        Float32MultiArray vel_message;
        vel_message.data = {float(WheelsVel(0, 0)),
                            float(WheelsVel(1, 0)), 
                            float(WheelsVel(2, 0)), 
                            float(WheelsVel(3, 0))};

        RCLCPP_DEBUG(this->get_logger(), "%s.", Message.c_str());
        publisher_->publish(vel_message);
    }

    void topic_callback(const Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Odom callback.");
        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;
        float z = msg->pose.pose.orientation.z;
        float w = msg->pose.pose.orientation.w;
        float phi = 2 * std::atan2(z, w);
        CurrentOdom << phi, x, y;
        compute_velocities();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EightTrajectory>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}