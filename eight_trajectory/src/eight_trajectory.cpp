#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <tuple>
#include <vector>

using namespace std::chrono_literals;
const double PI = 3.14159265359;

class EightTrajectoryController : public rclcpp::Node {
public:
  EightTrajectoryController() : Node("eight_trajectory_controller") {
    
    wheel_speed_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);

    
    timer_ = create_wall_timer(
        1000ms, std::bind(&EightTrajectoryController::timerCallback, this));

   
    callback_group_odom_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_odom_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectoryController::odomCallback, this,
                  std::placeholders::_1),
        options);

    
    initializeWaypoints();
  }

private:
  
  const double wheel_radius_ = 0.127; // 0.254/2
  const double robot_length_ = 0.25;  // 0.500/2
  const double robot_width_ = 0.274;  // 0.548/2

  // Control gains - exactly as in original
  const double k_rho_ = 0.3;
  const double k_alpha_ = 0.8;
  const double k_beta_ = 1.0;

  
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  double current_yaw_rad_;

  
  rclcpp::CallbackGroup::SharedPtr callback_group_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  
  std::list<std::tuple<double, double, double>> waypoints_ = {
      {0, 1, -1},        {0, 1, 1},    {0, 1, 1},    {1.5708, 1, -1},
      {-3.1415, -1, -1}, {0.0, -1, 1}, {0.0, -1, 1}, {0.0, -1, -1}};
  std::list<std::tuple<double, double, double>> ref_points_;

  void initializeWaypoints() {
    double cur_ref_phi = 0;
    double cur_ref_x = 0;
    double cur_ref_y = 0;

    for (const auto &[dphi, dx, dy] : waypoints_) {
      cur_ref_phi -= dphi;
      cur_ref_x += dx;
      cur_ref_y += dy;
      ref_points_.push_back(std::make_tuple(cur_ref_phi, cur_ref_x, cur_ref_y));
      RCLCPP_INFO(get_logger(), "Reference point: phi=%f, x=%f, y=%f",
                  cur_ref_phi, cur_ref_x, cur_ref_y);
    }
  }

  void timerCallback() {
    timer_->cancel();
    std::thread{std::bind(&EightTrajectoryController::execute, this)}.detach();
  }

  void execute() {
    auto message = std_msgs::msg::Float32MultiArray();
    const double error_tolerance = 0.01;

    while (!ref_points_.empty()) {
      const auto [phi_goal, x_goal, y_goal] = ref_points_.front();
      RCLCPP_INFO(get_logger(), "Moving to: phi=%f, x=%f, y=%f", phi_goal,
                  x_goal, y_goal);

      goTo(x_goal, y_goal, phi_goal, error_tolerance);
      std::this_thread::sleep_for(std::chrono::seconds(1));

      waypoints_.pop_front();
      ref_points_.pop_front();
    }

    RCLCPP_INFO(get_logger(), "No more waypoints");
    rclcpp::shutdown();
  }

  void goTo(double x_goal, double y_goal, double theta_goal_radian,
            double tolerance) {
    auto message = std_msgs::msg::Float32MultiArray();
    double rho = std::numeric_limits<double>::max();
    double theta_goal = normalizeAngle(theta_goal_radian);

    while (rho > tolerance) {
      double delta_x = x_goal - current_pos_.x;
      double delta_y = y_goal - current_pos_.y;
      rho = std::sqrt(delta_x * delta_x + delta_y * delta_y);

      double alpha = 0;
      double beta = normalizeAngle(-theta_goal - (alpha + current_yaw_rad_));
      double w = k_alpha_ * alpha + k_beta_ * beta;

      Eigen::Vector3d twist = velocityToTwist(w, delta_x, delta_y);
      auto wheel_speeds = twist2wheels(twist);

      message.data = wheel_speeds;
      wheel_speed_pub_->publish(message);

      std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
  }

  Eigen::Vector3d velocityToTwist(double dphi, double dx, double dy) {
    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, std::cos(current_yaw_rad_), std::sin(current_yaw_rad_), 0,
        -std::sin(current_yaw_rad_), std::cos(current_yaw_rad_);

    return R * Eigen::Vector3d(dphi, dx, dy);
  }

  std::vector<float> twist2wheels(const Eigen::Vector3d &twist) {
    Eigen::MatrixXd H(4, 3);
    double lr = (robot_length_ + robot_width_) / wheel_radius_;

    H << -lr, 1.0 / wheel_radius_, -1.0 / wheel_radius_, lr,
        1.0 / wheel_radius_, 1.0 / wheel_radius_, lr, 1.0 / wheel_radius_,
        -1.0 / wheel_radius_, -lr, 1.0 / wheel_radius_, 1.0 / wheel_radius_;

    Eigen::Vector4d speeds = H * twist;
    return {static_cast<float>(speeds(0)), static_cast<float>(speeds(1)),
            static_cast<float>(speeds(2)), static_cast<float>(speeds(3))};
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = getYawFromQuaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }

  double getYawFromQuaternion(double x, double y, double z, double w) {
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 matrix(q);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalizeAngle(double angle) {
    while (angle > PI)
      angle -= 2 * PI;
    while (angle < -PI)
      angle += 2 * PI;
    return angle;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EightTrajectoryController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}