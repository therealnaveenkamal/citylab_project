#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol") {
    lSubs = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
    vPubs = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    oth = 0.5;
    cTimer = this->create_wall_timer(std::chrono::milliseconds(125),
                                     std::bind(&Patrol::controlLoop, this));
    rev = false;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lSubs;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vPubs;
  double oth = 0.45;
  rclcpp::TimerBase::SharedPtr cTimer;
  bool rev;
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double front_range = msg->ranges[360];
    if (front_range < oth) {
      rev = true;
    } else {
      rev = false;
    }
  }

  void controlLoop() {
    geometry_msgs::msg::Twist velocity_command;

    if (rev) {
      velocity_command.linear.x = -0.1;
      velocity_command.angular.z = 0.0;
    } else {
      velocity_command.linear.x = 0.1;
      velocity_command.angular.z = 0.0;
    }

    vPubs->publish(velocity_command);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
