#include "cinterface/srv/get_direction.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using GetDirection = cinterface::srv::GetDirection;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol") {
    lSubs = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
    vPubs = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    oth = 0.6;
    cTimer = create_wall_timer(125ms, std::bind(&Patrol::controlLoop, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lSubs;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vPubs;

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("direction_client");
  rclcpp::Client<GetDirection>::SharedPtr client =
      node->create_client<GetDirection>("direction_service");

  double oth = 0.6;
  rclcpp::TimerBase::SharedPtr cTimer;
  geometry_msgs::msg::Twist velocity;

  int laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "STARTv2");

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data.angle_min = msg->angle_min;
    request->laser_data.angle_max = msg->angle_max;
    request->laser_data.angle_increment = msg->angle_increment;
    request->laser_data.time_increment = msg->time_increment;
    request->laser_data.scan_time = msg->scan_time;
    request->laser_data.range_min = msg->range_min;
    request->laser_data.range_max = msg->range_max;
    request->laser_data.ranges = msg->ranges;
    request->laser_data.intensities = msg->intensities;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      if (result->direction == "forward") {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned forward");
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.0;
        vPubs->publish(velocity);
      } else if (result->direction == "left") {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned left");
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.5;
        vPubs->publish(velocity);
      } else if (result->direction == "right") {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned right");
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.5;
        vPubs->publish(velocity);
      } else if (result->direction == "reverse-left") {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned reverse");
        velocity.linear.x = -0.1;
        velocity.angular.z = -0.5;
        vPubs->publish(velocity);
      } else if (result->direction == "reverse-right") {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned reverse");
        velocity.linear.x = -0.1;
        velocity.angular.z = 0.5;
        vPubs->publish(velocity);
      }
      return 0;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service /moving");

      return 0;
    }

    /*


        if (front < oth) {
          if (right > oth || left > oth) {
            RCLCPP_INFO(this->get_logger(), "Turning Right: '%f'", right);

            if (std::isinf(right)) {
              RCLCPP_INFO(this->get_logger(), "Reverse: '%f'", right);
              velocity.linear.x = 0;
              velocity.angular.z = -0.5;
              vPubs->publish(velocity);
            }

            velocity.linear.x = 0;
            velocity.angular.z = -0.5;
            vPubs->publish(velocity);
          } else if (right < 0.15) {
            velocity.linear.x = -0.1;
            velocity.angular.z = -0.5;
            vPubs->publish(velocity);
          } else {
            RCLCPP_INFO(this->get_logger(), "Reverse: '%f'", front);
            velocity.linear.x = -0.1;
            velocity.angular.z = -0.1;
            vPubs->publish(velocity);
          }
        } else {
          velocity.linear.x = 0.2;
          velocity.angular.z = 0.0;
          vPubs->publish(velocity);
        }

        */
  }

  void controlLoop() {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
