#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace camera_rospkg {

// Save raw video from camera to specified folder 
class VideoSaver : public rclcpp::Node {
public:
  explicit VideoSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  double fps_;
  std::string video_codec_;
  std::string output_folder_;
  int width_;
  int height_;


  // Capture
  cv::VideoWriter video_writer_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr video_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Methods
  void setupSubscriber();
  void videoCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

};

} // namespace camera_rospkg
