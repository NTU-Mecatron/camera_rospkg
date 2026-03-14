#pragma once

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace camera_rospkg {

// Lifecycle node: configure = open camera + create publishers; activate = start timer; deactivate = stop timer.
class CameraPublisher : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit CameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Stored parameters (declared in constructor, read in on_configure)
  std::string device_;
  std::string frame_id_;
  int width_;
  int height_;
  double fps_;
  bool rectify_;


  // Capture
  cv::VideoCapture cap_;

  // Publishers (lifecycle-managed)
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  // Camera info / calibration
  sensor_msgs::msg::CameraInfo curr_cinfo_;

  // Rectification maps (for plumb_bob or fisheye)
  cv::Mat map1_, map2_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Helpers
  void loadCalibration(const std::string & calibration_url);
  void openCamera();
  void buildRectifyMaps();
  void timerCb();
  void teardown();  // shared cleanup logic
};

} // namespace camera_rospkg
