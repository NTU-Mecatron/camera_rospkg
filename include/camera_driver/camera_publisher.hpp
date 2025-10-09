#pragma once


#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

#include <opencv2/opencv.hpp>

namespace camera_driver {

// Publishes raw Image + CameraInfo, with optional undistort/rectify from YAML
// Foxglove H.264 is provided by the installed image_transport plugin
class CameraPublisher : public rclcpp::Node {
public:
  explicit CameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Parameters
  std::string device_;              // "/dev/video0" or "0"
  std::string frame_id_;            // TF frame
  int width_;                       // capture width
  int height_;                      // capture height
  double fps_;                      // capture fps
  bool rectify_;                    // enable undistort/rectify
  std::string calibration_url_;     // "file:///abs/path/calibration.yaml"

  // Capture
  cv::VideoCapture cap_;

  // Publishers
  image_transport::Publisher img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  // Camera info / calibration
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo_mgr_;
  sensor_msgs::msg::CameraInfo curr_cinfo_;  // cached, updated on load

  // Rectification maps (for plumb_bob or fisheye)
  cv::Mat map1_, map2_;
  bool maps_ready_ = false;
  bool is_fisheye_ = false;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Methods
  void openCamera();
  void setupPublishers();
  void loadCalibration();
  void buildRectifyMaps();  // from curr_cinfo_ into map1_/map2_
  void timerCb();

  static bool isDigits(const std::string & s);
};

} // namespace camera_driver
