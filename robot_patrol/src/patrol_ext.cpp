#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol") {
    lSubs = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
    vPubs = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    oth = 0.4;
    cTimer = create_wall_timer(125ms, std::bind(&Patrol::controlLoop, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lSubs;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vPubs;
  double oth;
  rclcpp::TimerBase::SharedPtr cTimer;
  geometry_msgs::msg::Twist velocity;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double front = msg->ranges[360];
    double left = msg->ranges[180];
    double right = msg->ranges[540];

    RCLCPP_INFO(this->get_logger(), "Front: '%f', Left: '%f', Right: '%f' ",
                front, left, right);
    if (front < oth || std::isinf(front)) {
      if (left > oth && left > right) {
        RCLCPP_INFO(this->get_logger(), "Left: '%f', Right: '%f'", left, right);
        velocity.linear.x = -0.1;
        velocity.angular.z = 0.5;
        vPubs->publish(velocity);
      } else if (right > oth) {
        RCLCPP_INFO(this->get_logger(), "Turning Right: '%f'", right);
        velocity.linear.x = -0.1;
        velocity.angular.z = -0.5;
        vPubs->publish(velocity);
      } else {
        RCLCPP_INFO(this->get_logger(), "Reverse: '%f'", front);
        velocity.linear.x = -0.3;
        velocity.angular.z = 0.0;
        vPubs->publish(velocity);
      }
    } else {
      velocity.linear.x = 0.3;
      velocity.angular.z = 0.0;
      vPubs->publish(velocity);
    }
  }

  void controlLoop() {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
