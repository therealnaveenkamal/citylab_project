#include "cinterface/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>

class GoToPose : public rclcpp::Node {
public:
  using GTP = cinterface::action::GoToPose;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<GTP>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GTP>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    odom_subscription = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odomCallback, this, _1));

    Kp_ = 0.4;
    Ki_ = 0.02;
    Kd_ = 0.22;
    prev_error_ = 0;
    int_error_ = 0;

    MAX_LINEAR_SPEED = 0.08;
    MAX_ANGULAR_SPEED = 0.65;
  }

private:
  rclcpp_action::Server<GTP>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  geometry_msgs::msg::Pose2D desired_pos_ = geometry_msgs::msg::Pose2D();
  geometry_msgs::msg::Pose2D current_pos_ = geometry_msgs::msg::Pose2D();

  double Kp_;
  double Ki_;
  double Kd_;
  double prev_error_;
  double int_error_;

  double MAX_LINEAR_SPEED;
  double MAX_ANGULAR_SPEED;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GTP::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal with coords: x=%f, y=%f, theta=%f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancelling goal");
    (void)goal_handle;
    geometry_msgs::msg::Twist stopbot;
    stopbot.linear.x = 0.0;
    stopbot.angular.z = 0.0;
    publisher_->publish(stopbot);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GTP::Result>();
    auto move = geometry_msgs::msg::Twist();
    auto feedback = std::make_shared<GTP::Feedback>();
    auto &feedback_data = feedback->current_pos;

    rclcpp::Rate loop_rate(10);
    double tolerance = 0.1;

    while (rclcpp::ok()) {
      double current_x = current_pos_.x;
      double current_y = current_pos_.y;

      double desired_x = goal->goal_pos.x;
      double desired_y = goal->goal_pos.y;

      feedback_data.x = current_x;
      feedback_data.y = current_y;
      feedback_data.theta = current_pos_.theta;

      double dx = desired_x - current_x;
      double dy = desired_y - current_y;

      double d_error = std::sqrt(std::pow(desired_x - current_x, 2) +
                                 std::pow(desired_y - current_y, 2));

      if (d_error <= tolerance) {
        result->status = "Goal achieved.";
        move.linear.x = 0.0;
        move.angular.z = 0.0;
        publisher_->publish(move);
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal achieved");

        while (rclcpp::ok()) {
          loop_rate.sleep();
        }
      } else {
        double tan_inverse = atan2(dy, dx);
        double a_error = tan_inverse - current_pos_.theta;

        while (a_error > 3.145) {
          a_error -= 2 * 3.145;
        }
        while (a_error < -3.145) {
          a_error += 2 * 3.145;
        }

        double angular_speed =
            Kp_ * a_error + Ki_ * int_error_ + Kd_ * (a_error - prev_error_);
        prev_error_ = a_error;
        int_error_ += a_error;

        angular_speed = std::max(-MAX_ANGULAR_SPEED,
                                 std::min(angular_speed, MAX_ANGULAR_SPEED));

        move.linear.x = MAX_LINEAR_SPEED;
        move.angular.z = angular_speed;
        publisher_->publish(move);

        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Error: %f", d_error);
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", current_x, current_y);
      }

      loop_rate.sleep();
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    tf2::Quaternion orientation;
    tf2::fromMsg(msg->pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    current_pos_.x = x;
    current_pos_.y = y;
    current_pos_.theta = yaw;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);

  std::thread executor_thread([&executor]() { executor.spin(); });
  executor_thread.join();

  return 0;
}