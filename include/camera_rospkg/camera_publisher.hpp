#pragma once


#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#endif
#include <camera_info_manager/camera_info_manager.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace camera_rospkg {

// Publishes raw Image + compressed Image + CameraInfo, with optional undistort/rectify from YAML
class CameraPublisher : public rclcpp::Node {
public:
  explicit CameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Parameters
  std::string frame_id_;            // TF frame
  int width_;                       // capture width
  int height_;                      // capture height
  bool rectify_;                    // enable undistort/rectify

  // Capture
  cv::VideoCapture cap_;

  // Publishers
  image_transport::Publisher img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  // Camera info / calibration
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo_mgr_;
  sensor_msgs::msg::CameraInfo curr_cinfo_;  // cached, loaded once at startup

  // Rectification maps (for plumb_bob or fisheye)
  cv::Mat map1_, map2_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Methods
  void openCamera(const std::string& device, double fps);
  void loadCalibration(const std::string& calibration_url);
  void buildRectifyMaps();  // from curr_cinfo_ into map1_/map2_
  void timerCb();
};

} // namespace camera_rospkg
