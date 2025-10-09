#include "camera_driver/camera_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_driver::CameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
