#include "camera_rospkg/camera_publisher.hpp"

#include <algorithm>
#include <string>
#include <chrono>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace camera_rospkg {

CameraPublisher::CameraPublisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("camera_publisher", options)
{
  // Declare parameters with defaults
  std::string device = declare_parameter<std::string>("device", "/dev/video4");
  frame_id_        = declare_parameter<std::string>("frame_id", "camera_optical_frame");
  width_           = declare_parameter<int>("width", 640);
  height_          = declare_parameter<int>("height", 480);
  double fps       = declare_parameter<double>("fps", 30.0);
  rectify_         = declare_parameter<bool>("rectify", true);
  std::string calibration_url = declare_parameter<std::string>("calibration_url", "");

  const std::string camera_name = declare_parameter<std::string>("camera_name", "camera");
  cinfo_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);

  // Load calibration
  loadCalibration(calibration_url);

  // Open camera and set properties
  openCamera(device, fps);

  // Create publishers (raw image + camera info)
  img_pub_ = image_transport::create_publisher(this, "image");
  cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());

  // Periodic capture/publish timer
  using namespace std::chrono_literals;
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, fps));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { timerCb(); });

  RCLCPP_INFO(get_logger(), "camera_publisher started: device=%s %dx%d @ %.1fHz rectify=%s",
              device.c_str(), width_, height_, fps, rectify_ ? "true" : "false");
}

void CameraPublisher::openCamera(const std::string& device, double fps)
{
  // Detect video file by extension
  auto dot = device.rfind('.');
  std::string ext = (dot != std::string::npos) ? device.substr(dot) : "";
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  bool is_file = (ext == ".mp4" || ext == ".avi" || ext == ".mov" || ext == ".mkv");
  int backend = is_file ? cv::CAP_ANY : cv::CAP_V4L2;

  // Open by index ("0") or path ("/dev/video0", "test.mp4")
  bool is_index = !device.empty() && std::all_of(device.begin(), device.end(), ::isdigit);
  if (is_index) {
    cap_.open(std::stoi(device), backend);
  } else {
    cap_.open(device, backend);
  }

  if (!cap_.isOpened()) {
    throw std::runtime_error("Failed to open device: " + device);
  }

  // Configure hardware capture properties (not applicable to video files)
  if (!is_file) {
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps);
  }

  RCLCPP_INFO(get_logger(), "Opened %s '%s'", is_file ? "file" : "camera", device.c_str());
}

void CameraPublisher::loadCalibration(const std::string& calibration_url)
{
  if (calibration_url.empty()) {
    RCLCPP_WARN(get_logger(), "No calibration_url provided. Publishing uncalibrated CameraInfo.");
    curr_cinfo_ = sensor_msgs::msg::CameraInfo();
    curr_cinfo_.distortion_model = "plumb_bob";
    curr_cinfo_.d.resize(5, 0.0);
    return;
  }

  std::string url = calibration_url;
  if (url.rfind("file://", 0) != 0 && url.rfind("package://", 0) != 0) {
    url = "file://" + url;
  }

  if (!cinfo_mgr_->validateURL(url)) {
    throw std::runtime_error("Invalid calibration URL: " + url);
  }
  if (!cinfo_mgr_->loadCameraInfo(url)) {
    throw std::runtime_error("Failed to load calibration: " + url);
  }

  curr_cinfo_ = cinfo_mgr_->getCameraInfo();
  RCLCPP_INFO(get_logger(), "Loaded calibration from %s", url.c_str());

  if ((int)curr_cinfo_.width != width_ || (int)curr_cinfo_.height != height_) {
    RCLCPP_WARN(get_logger(),
      "Calibration resolution (%ux%u) != requested (%dx%d). Rectification maps may be inaccurate.",
      curr_cinfo_.width, curr_cinfo_.height, width_, height_);
  }

  if (rectify_) {
    buildRectifyMaps();
  }
}

void CameraPublisher::buildRectifyMaps()
{
  // Extract calibration matrices
  // K (3x3), D (length 4/5/8), R (3x3), P (3x4). Use P[0..2][0..2] as new camera matrix if available.
  cv::Mat K(3, 3, CV_64F, (void*)curr_cinfo_.k.data());  // camera matrix
  cv::Mat D = cv::Mat(curr_cinfo_.d).clone();            // distortion coeffs
  cv::Mat R = cv::Mat(3, 3, CV_64F, (void*)curr_cinfo_.r.data());  // rectification
  cv::Mat P(3, 4, CV_64F, (void*)curr_cinfo_.p.data());            // projection

  // New camera matrix: prefer P's left 3x3 if provided, else K
  cv::Mat newK = P(cv::Rect(0, 0, 3, 3)).clone();
  if (cv::countNonZero(newK) == 0) {
    newK = K.clone();
  }

  const cv::Size img_size(width_, height_);

  // Decide distortion model
  bool is_fisheye = (curr_cinfo_.distortion_model == "equidistant");

  if (!is_fisheye) {
    // Standard pinhole ("plumb_bob")
    cv::initUndistortRectifyMap(
      K, D, R, newK, img_size, CV_32FC1, map1_, map2_);
  } else {
    // Fisheye (equidistant); D must be 4 coeffs
    cv::Mat K32, newK32, R32;
    K.convertTo(K32, CV_32F); newK.convertTo(newK32, CV_32F); R.convertTo(R32, CV_32F);
    cv::fisheye::initUndistortRectifyMap(
      K32, D, R32, newK32, img_size, CV_32FC1, map1_, map2_);
  }

  RCLCPP_INFO(get_logger(), "Rectification maps built (%s).",
              is_fisheye ? "fisheye/equidistant" : "pinhole/plumb_bob");
}

void CameraPublisher::timerCb()
{
  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read frame");
    return;
  }

  // Warn if driver returns unexpected resolution (silent resize would break calibration)
  if (frame.cols != width_ || frame.rows != height_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Driver returned %dx%d, expected %dx%d. Check device settings.",
      frame.cols, frame.rows, width_, height_);
  }

  // Rectify if enabled
  if (rectify_) {
    cv::Mat rectified;
    cv::remap(frame, rectified, map1_, map2_, cv::INTER_LINEAR);
    frame = rectified;
  }

  // Compose ROS messages
  const rclcpp::Time stamp = now();

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id_;

  auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
  img_pub_.publish(img_msg);

  // CameraInfo: use stored calibration; update header + size to match frame
  auto cinfo = curr_cinfo_;
  cinfo.header.stamp = stamp;
  cinfo.header.frame_id = frame_id_;
  cinfo.width  = static_cast<uint32_t>(frame.cols);
  cinfo.height = static_cast<uint32_t>(frame.rows);
  cinfo_pub_->publish(cinfo);
}

} // namespace camera_rospkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera_rospkg::CameraPublisher)
