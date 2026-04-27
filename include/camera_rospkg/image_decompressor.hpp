#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#endif

namespace camera_rospkg {

using namespace sensor_msgs::msg;

// Component node to decompress image/compressed and republish as image/raw.
class ImageDecompressor : public rclcpp::Node {
public:
  explicit ImageDecompressor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onImage(CompressedImage::ConstSharedPtr msg);

  rclcpp::Subscription<CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  std::string encoding_;
};

} // namespace camera_rospkg