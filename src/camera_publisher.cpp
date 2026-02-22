#include "camera_rospkg/camera_publisher.hpp"

#include <algorithm>
#include <string>
#include <chrono>

namespace camera_rospkg {

CameraPublisher::CameraPublisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("camera_publisher", options),
    nh_(std::shared_ptr<CameraPublisher>(this, [](auto *) {})),
    it_(nh_),
    img_pub_(it_.advertise("image", 10))
{
  // Declare parameters with defaults
  std::string device = declare_parameter<std::string>("device", "/dev/video4");
  frame_id_        = declare_parameter<std::string>("frame_id", "camera_optical_frame");
  width_           = declare_parameter<int>("width", 640);
  height_          = declare_parameter<int>("height", 480);
  double fps       = declare_parameter<double>("fps", 30.0);
  rectify_         = declare_parameter<bool>("rectify", true);
  std::string calibration_url = declare_parameter<std::string>("calibration_url", "config/calibration.yaml");

  // CameraInfoManager needs a "camera_name" (used as namespace inside YAML)
  const std::string camera_name = declare_parameter<std::string>("camera_name", "camera");
  cinfo_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);

  // Load calibration immediately (optional if empty URL)
  loadCalibration(calibration_url);

  // Open camera and set properties
  openCamera(device, fps);

  // Create publishers (raw image + camera info)
  setupPublishers();

  // Periodic capture/publish timer
  using namespace std::chrono_literals;
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, fps));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CameraPublisher::timerCb, this));

  RCLCPP_INFO(get_logger(), "camera_publisher started: device=%s %dx%d @ %.1fHz rectify=%s",
              device.c_str(), width_, height_, fps, rectify_ ? "true" : "false");
}

bool CameraPublisher::isDigits(const std::string & s)
{
  return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit);
}

void CameraPublisher::openCamera(const std::string& device, double fps)
{
  std::string lower_device = device;
  std::transform(lower_device.begin(), lower_device.end(), lower_device.begin(), ::tolower);
  
  bool is_video_file = (lower_device.length() >= 4 && (
                        lower_device.substr(lower_device.length() - 4) == ".mp4" ||
                        lower_device.substr(lower_device.length() - 4) == ".avi" ||
                        lower_device.substr(lower_device.length() - 4) == ".mov"));

  try {
    int api_preference = is_video_file ? cv::CAP_ANY : cv::CAP_V4L2;

    if (is_video_file) {
      RCLCPP_INFO(get_logger(), "Detected video file extension; using default backend.");
    } else {
      RCLCPP_INFO(get_logger(), "Requesting V4L2 backend for camera device.");
    }
    
    if (isDigits(device)) {
      cap_.open(std::stoi(device), api_preference);
    } else {
      cap_.open(device, api_preference);
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Exception opening camera: %s", e.what());
    throw;
  }

  if (!cap_.isOpened()) {
    RCLCPP_FATAL(get_logger(), "Failed to open camera device '%s'", device.c_str());
    throw std::runtime_error("camera open failed");
  }

  if (!is_video_file) {
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!cap_.set(cv::CAP_PROP_FOURCC, fourcc)) {
      RCLCPP_WARN(get_logger(), "Failed to set pixel format to MJPG");
    } else {
      RCLCPP_INFO(get_logger(), "Successfully set pixel format to MJPG");
    }

    // Best-effort property set (driver may clamp)
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps);
  }
}

void CameraPublisher::setupPublishers()
{
  // Creates multiple topics with different transport if using ffmpeg plugin
  // image_raw/ffmpeg for H.264, image_raw for raw
  // Using relative topic names to allow for proper namespacing
  // img_pub_ = image_transport::create_publisher(this, "image_raw");

  // standard CameraInfo publisher
  cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());
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

  // camera_info_manager expects a URL; prepend file:// if you gave a bare path
  std::string url = calibration_url;
  if (url.rfind("file://", 0) != 0 && url.rfind("package://", 0) != 0) {
    url = "file://" + url;
  }

  if (!cinfo_mgr_->validateURL(url)) {
    RCLCPP_ERROR(get_logger(), "Invalid calibration URL: %s", url.c_str());
    throw std::runtime_error("invalid calibration url");
  }

  if (!cinfo_mgr_->loadCameraInfo(url)) {
    RCLCPP_ERROR(get_logger(), "Failed to load calibration from %s", url.c_str());
    throw std::runtime_error("calibration load failed");
  }

  curr_cinfo_ = cinfo_mgr_->getCameraInfo();

  // Sanity: check size
  if ((int)curr_cinfo_.width != width_ || (int)curr_cinfo_.height != height_) {
    RCLCPP_WARN(get_logger(),
      "Calibration resolution (%ux%u) != requested (%dx%d). Proceeding, but rectification maps may be inaccurate.",
      curr_cinfo_.width, curr_cinfo_.height, width_, height_);
  }

  // Precompute rectification maps if requested
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

  maps_ready_ = true;
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

  // Ensure captured frame matches expected size (some drivers ignore set())
  if (frame.cols != width_ || frame.rows != height_) {
    cv::resize(frame, frame, cv::Size(width_, height_));
  }

  // Rectify if enabled and maps are ready
  if (rectify_ && maps_ready_) {
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
