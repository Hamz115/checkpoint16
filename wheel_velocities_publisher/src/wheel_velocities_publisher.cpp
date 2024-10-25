#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using namespace std::chrono_literals;

class WheelVelocities : public rclcpp::Node {
private:
  struct RobotParams {
    float wheel_radius;
    float wheel_base_width;
    float wheel_base_length;
  };

  struct VelocityCommand {
    float x;
    float y;
    float theta;
    std::string description;
  };

  
  rclcpp::Publisher<Float32MultiArray>::SharedPtr velocity_publisher_;
  RobotParams robot_params_;
  std::vector<VelocityCommand> motion_sequence_;
  Eigen::MatrixXd holonomic_matrix_;

  
  void initializeRobotParams() {
    robot_params_.wheel_radius = 0.05;         // r
    robot_params_.wheel_base_width = 0.134845; // w
    robot_params_.wheel_base_length = 0.085;   // l

    // Pre-compute holonomic matrix
    float l = robot_params_.wheel_base_length;
    float w = robot_params_.wheel_base_width;
    holonomic_matrix_.resize(4, 3);
    holonomic_matrix_ << -l - w, 1, -1, l + w, 1, 1, l + w, 1, -1, -l - w, 1, 1;
  }

 
  void initializeMotionSequence() {
    float lin_x = 0.0, lin_y = 0.0, ang_z = 0.0;
    this->get_parameter("linear_x", lin_x);
    this->get_parameter("linear_y", lin_y);
    this->get_parameter("angular_z", ang_z);

    motion_sequence_ = {{0.0, lin_x, 0.0, "Move forward"},
                        {0.0, -lin_x, 0.0, "Move backward"},
                        {0.0, 0.0, lin_y, "Move left"},
                        {0.0, 0.0, -lin_y, "Move right"},
                        {ang_z, 0.0, 0.0, "Turn clockwise"},
                        {-ang_z, 0.0, 0.0, "Turn counter-clockwise"},
                        {0.0, 0.0, 0.0, "Stop"}};
  }

  
  void publishWheelVelocities(const VelocityCommand &cmd) {
    Eigen::MatrixXd command_vector(3, 1);
    command_vector << cmd.x, cmd.y, cmd.theta;

    Eigen::MatrixXd wheel_velocities =
        (1.0 / robot_params_.wheel_radius) * holonomic_matrix_ * command_vector;

    Float32MultiArray velocity_msg;
    velocity_msg.data.resize(4);
    for (int i = 0; i < 4; ++i) {
      velocity_msg.data[i] = static_cast<float>(wheel_velocities(i, 0));
    }

    RCLCPP_INFO(this->get_logger(), "%s.", cmd.description.c_str());
    velocity_publisher_->publish(velocity_msg);
  }

  // Execute motion sequence
  void executeMotionSequence() {
    rclcpp::sleep_for(3s); // Initial delay

    for (const auto &motion : motion_sequence_) {
      publishWheelVelocities(motion);
      rclcpp::sleep_for(3s);
    }
  }

public:
  WheelVelocities() : Node("wheel_velocities_publisher") {
    // Initialize parameters
    this->declare_parameter<float>("linear_x", 0.5);
    this->declare_parameter<float>("linear_y", 0.5);
    this->declare_parameter<float>("angular_z", 0.5);

    // Create publisher
    velocity_publisher_ =
        this->create_publisher<Float32MultiArray>("/wheel_speed", 10);

    // Initialize robot configuration
    initializeRobotParams();
    initializeMotionSequence();

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node.");

    // Start motion sequence
    executeMotionSequence();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocities>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}