#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <memory>

using Twist = geometry_msgs::msg::Twist;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;

class HolonomicKinematics {
public:
  struct RobotParameters {
    float wheel_radius;
    float wheel_base_width;
    float wheel_base_length;

    RobotParameters(float r, float w, float l)
        : wheel_radius(r), wheel_base_width(w), wheel_base_length(l) {
      initializeHolonomicMatrix();
    }

  private:
    friend class HolonomicKinematics;
    Eigen::MatrixXd holonomic_matrix;

    void initializeHolonomicMatrix() {
      holonomic_matrix.resize(4, 3);
      holonomic_matrix << -wheel_base_length - wheel_base_width, 1, -1,
          wheel_base_length + wheel_base_width, 1, 1,
          wheel_base_length + wheel_base_width, 1, -1,
          -wheel_base_length - wheel_base_width, 1, 1;
    }
  };

  explicit HolonomicKinematics(const RobotParameters &params)
      : params_(params), pinv_holonomic_matrix_(
                             calculatePseudoInverse(params_.holonomic_matrix)) {
  }

  Twist convertWheelVelocitiesToTwist(
      const std::vector<float> &wheel_velocities) const {
    Eigen::MatrixXd wheel_vel_matrix(4, 1);
    for (size_t i = 0; i < 4; ++i) {
      wheel_vel_matrix(i, 0) = wheel_velocities[i];
    }

    Eigen::MatrixXd cmd_vel =
        pinv_holonomic_matrix_ * params_.wheel_radius * wheel_vel_matrix;

    Twist twist_msg;
    twist_msg.angular.z = static_cast<float>(cmd_vel(0, 0));
    twist_msg.linear.x = static_cast<float>(cmd_vel(1, 0));
    twist_msg.linear.y = static_cast<float>(cmd_vel(2, 0));

    return twist_msg;
  }

private:
  RobotParameters params_;
  Eigen::MatrixXd pinv_holonomic_matrix_;

  static Eigen::MatrixXd calculatePseudoInverse(const Eigen::MatrixXd &matrix) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU |
                                                      Eigen::ComputeFullV);
    return svd.solve(Eigen::MatrixXd::Identity(matrix.rows(), matrix.rows()));
  }
};

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel()
      : Node("kinematics_model_publisher"),
        kinematics_(std::make_unique<HolonomicKinematics>(
            HolonomicKinematics::RobotParameters(0.05, 0.134845, 0.085))) {
    initializeNode();
  }

private:
  std::unique_ptr<HolonomicKinematics> kinematics_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<Float32MultiArray>::SharedPtr wheel_speed_subscriber_;

  void initializeNode() {
    RCLCPP_INFO(get_logger(), "Initialized kinematics model publisher node.");

    cmd_vel_publisher_ = create_publisher<Twist>("/cmd_vel", 10);

    wheel_speed_subscriber_ = create_subscription<Float32MultiArray>(
        "/wheel_speed", 10,
        std::bind(&KinematicModel::wheelSpeedCallback, this,
                  std::placeholders::_1));
  }

  void wheelSpeedCallback(const Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_ERROR(get_logger(), "Invalid wheel velocities array size: %zu",
                   msg->data.size());
      return;
    }

    auto twist_msg = kinematics_->convertWheelVelocitiesToTwist(msg->data);

    logVelocities(twist_msg);
    cmd_vel_publisher_->publish(twist_msg);
  }

  void logVelocities(const Twist &twist_msg) {
    RCLCPP_DEBUG(get_logger(), "Received velocities:");
    RCLCPP_DEBUG(get_logger(), "Angular Z: %.2f", twist_msg.angular.z);
    RCLCPP_DEBUG(get_logger(), " Linear X: %.2f", twist_msg.linear.x);
    RCLCPP_DEBUG(get_logger(), " Linear Y: %.2f", twist_msg.linear.y);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicModel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}