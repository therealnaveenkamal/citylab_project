#include "cinterface/srv/get_direction.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using GetDirection = cinterface::srv::GetDirection;

class ServiceNodeTest : public rclcpp::Node {
public:
  ServiceNodeTest() : Node("patrol_test") {
    lSubs = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ServiceNodeTest::laserCallback, this,
                  std::placeholders::_1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lSubs;

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("direction_client");
  rclcpp::Client<GetDirection>::SharedPtr client =
      node->create_client<GetDirection>("direction_service");

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
      if (result->direction == "forward" || result->direction == "left" ||
          result->direction == "right") {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Received direction from server: %s",
                    result->direction.c_str());
      } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Received unknown text from server");
      }
      return 0;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service /moving");

      return 0;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceNodeTest>());
  rclcpp::shutdown();
  return 0;
}
