#include "cinterface/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

using GetDirection = cinterface::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_server") {

    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::moving_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  void moving_callback(const std::shared_ptr<GetDirection::Request> request,
                       const std::shared_ptr<GetDirection::Response> response) {
    // RCLCPP_INFO(this->get_logger(), "START");

    double total_dist_sec_right = 0;
    double total_dist_sec_front = 0;
    double total_dist_sec_left = 0;

    for (size_t i = 0; i < request->laser_data.ranges.size(); ++i) {
      float angle = request->laser_data.angle_min +
                    i * request->laser_data.angle_increment;

      if (angle <= 1.57 && angle >= 0.524) {
        total_dist_sec_left += request->laser_data.ranges[i];
      } else if (angle < 0.524 && angle >= -0.524) {
        total_dist_sec_front += request->laser_data.ranges[i];
      } else if (angle < -0.524 && angle >= -1.57) {
        total_dist_sec_right += request->laser_data.ranges[i];
      }
    }

    if (total_dist_sec_front > total_dist_sec_left &&
        total_dist_sec_front > total_dist_sec_right) {

      response->direction = "forward";
    } else if (total_dist_sec_right > total_dist_sec_left &&
               total_dist_sec_right > total_dist_sec_front) {

      response->direction = "right";
    } else {
      response->direction = "right";
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}