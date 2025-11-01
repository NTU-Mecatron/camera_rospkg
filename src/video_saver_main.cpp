#include "camera_rospkg/video_saver.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_rospkg::VideoSaver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
