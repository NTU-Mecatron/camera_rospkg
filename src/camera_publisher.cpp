#include "camera_rospkg/camera_publisher.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>
#include <chrono>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

namespace camera_rospkg {

CameraPublisher::CameraPublisher(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("camera_publisher", options)
{
  declare_parameter<std::string>("device", "/dev/video0");
  declare_parameter<std::string>("frame_id", "camera_optical_frame");
  declare_parameter<int>("width", 640);
  declare_parameter<int>("height", 480);
  declare_parameter<double>("fps", 30.0);
  declare_parameter<bool>("rectify", true);
  declare_parameter<std::string>("calibration_url", "");
}

CameraPublisher::CallbackReturn CameraPublisher::on_configure(const rclcpp_lifecycle::State &)
{
  device_      = get_parameter("device").as_string();
  frame_id_    = get_parameter("frame_id").as_string();
  width_       = get_parameter("width").as_int();
  height_      = get_parameter("height").as_int();
  fps_         = get_parameter("fps").as_double();
  rectify_     = get_parameter("rectify").as_bool();

  try {
    loadCalibration(get_parameter("calibration_url").as_string());
    openCamera();
  } catch (const std::exception & e) {
    teardown();
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  img_pub_        = create_publisher<Image>("image/raw", rclcpp::SensorDataQoS());
  compressed_pub_ = create_publisher<CompressedImage>("image/compressed", rclcpp::SensorDataQoS());
  cinfo_pub_      = create_publisher<CameraInfo>("camera_info", rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "Configured: device=%s %dx%d @ %.1fHz rectify=%s",
    device_.c_str(), width_, height_, fps_, rectify_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

CameraPublisher::CallbackReturn CameraPublisher::on_activate(const rclcpp_lifecycle::State & state)
{
  if (!img_pub_ || !compressed_pub_ || !cinfo_pub_) {
    RCLCPP_ERROR(get_logger(), "Cannot activate before publishers are configured.");
    return CallbackReturn::FAILURE;
  }
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Cannot activate before the camera device is open.");
    return CallbackReturn::FAILURE;
  }
  
  // This will automatically activate all lifecycle publishers
  LifecycleNode::on_activate(state);

  // Start publishing thread
  should_publish_.store(true);
  publishing_thread_ = std::thread([this]() { publishingThreadLoop(); });

  RCLCPP_INFO(get_logger(), "Activated: publishing started.");
  return CallbackReturn::SUCCESS;
}

CameraPublisher::CallbackReturn CameraPublisher::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Stop publishing thread
  should_publish_.store(false);
  if (publishing_thread_.joinable()) {
    publishing_thread_.join();
  }

  // This will automatically deactivate all lifecycle publishers
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(get_logger(), "Deactivated: publishing paused.");
  return CallbackReturn::SUCCESS;
}

CameraPublisher::CallbackReturn CameraPublisher::on_cleanup(const rclcpp_lifecycle::State &)
{
  teardown(); 
  RCLCPP_INFO(get_logger(), "Cleaned up."); 
  return CallbackReturn::SUCCESS;
}

CameraPublisher::CallbackReturn CameraPublisher::on_shutdown(const rclcpp_lifecycle::State & state)
{
  teardown();
  RCLCPP_INFO(get_logger(), "Shut down from state: %s", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

void CameraPublisher::teardown()
{
  // Stop publishing thread
  should_publish_.store(false);
  if (publishing_thread_.joinable()) {
    publishing_thread_.join();
  }

  img_pub_.reset();
  compressed_pub_.reset();
  cinfo_pub_.reset();
  cap_.release();
  curr_cinfo_ = CameraInfo{};
  map1_ = cv::Mat{};
  map2_ = cv::Mat{};
}

void CameraPublisher::openCamera()
{
  auto dot = device_.rfind('.');
  std::string ext = (dot != std::string::npos) ? device_.substr(dot) : "";
  std::transform(ext.begin(), ext.end(), ext.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const bool is_file = (ext == ".mp4" || ext == ".avi" || ext == ".mov" || ext == ".mkv");
  const int backend  = is_file ? cv::CAP_ANY : cv::CAP_V4L2;

  const bool is_index = !device_.empty() && std::all_of(
    device_.begin(), device_.end(),
    [](unsigned char c) { return std::isdigit(c) != 0; });
  if (is_index) {
    cap_.open(std::stoi(device_), backend);
  } else {
    cap_.open(device_, backend);
  }

  if (!cap_.isOpened()) {
    throw std::runtime_error("Failed to open device: " + device_);
  }

  if (!is_file) {
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);
  }

  RCLCPP_INFO(get_logger(), "Opened %s '%s'", is_file ? "file" : "camera", device_.c_str());
}

void CameraPublisher::loadCalibration(const std::string& calibration_url)
{
  if (calibration_url.empty()) {
    RCLCPP_WARN(get_logger(), "No calibration_url provided. Publishing uncalibrated CameraInfo.");
    curr_cinfo_ = CameraInfo{};
    curr_cinfo_.distortion_model = "plumb_bob";
    curr_cinfo_.d.resize(5, 0.0);
    return;
  }

  std::string path = calibration_url;
  const std::string file_prefix = "file://";
  if (path.rfind(file_prefix, 0) == 0) {
    path = path.substr(file_prefix.size());
  } else if (path.rfind("package://", 0) == 0) {
    throw std::runtime_error("package:// URLs are not supported. Use an absolute file path.");
  }

  if (!std::ifstream(path).good()) {
    throw std::runtime_error("Calibration file not found: " + path);
  }

  try {
    YAML::Node yaml = YAML::LoadFile(path);

    curr_cinfo_ = CameraInfo{};
    curr_cinfo_.width  = yaml["image_width"].as<uint32_t>();
    curr_cinfo_.height = yaml["image_height"].as<uint32_t>();
    curr_cinfo_.distortion_model = yaml["distortion_model"].as<std::string>();

    const auto fill_array = [&](const char* key, auto& arr) {
      auto data = yaml[key]["data"];
      for (size_t i = 0; i < arr.size(); ++i) {
        arr[i] = data[i].as<double>();
      }
    };
    fill_array("camera_matrix",        curr_cinfo_.k);
    fill_array("rectification_matrix", curr_cinfo_.r);
    fill_array("projection_matrix",    curr_cinfo_.p);

    auto d_data = yaml["distortion_coefficients"]["data"];
    curr_cinfo_.d.resize(d_data.size());
    for (size_t i = 0; i < d_data.size(); ++i) {
      curr_cinfo_.d[i] = d_data[i].as<double>();
    }

  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("Failed to parse calibration YAML: ") + e.what());
  }

  RCLCPP_INFO(get_logger(), "Loaded calibration from %s", path.c_str());

  if (static_cast<int>(curr_cinfo_.width) != width_ || static_cast<int>(curr_cinfo_.height) != height_) {
    RCLCPP_WARN(get_logger(),
      "Calibration resolution (%ux%u) != requested (%dx%d). Rectification maps may be inaccurate.",
      curr_cinfo_.width, curr_cinfo_.height, width_, height_);
  }

  if (rectify_) buildRectifyMaps();
}

void CameraPublisher::buildRectifyMaps()
{
  cv::Mat K(3, 3, CV_64F, (void*)curr_cinfo_.k.data());
  cv::Mat D = cv::Mat(curr_cinfo_.d).clone();
  cv::Mat R(3, 3, CV_64F, (void*)curr_cinfo_.r.data());
  cv::Mat P(3, 4, CV_64F, (void*)curr_cinfo_.p.data());

  cv::Mat newK = P(cv::Rect(0, 0, 3, 3)).clone();
  if (cv::countNonZero(newK) == 0) newK = K.clone();

  const cv::Size img_size(width_, height_);
  const bool is_fisheye = (curr_cinfo_.distortion_model == "equidistant");

  if (!is_fisheye) {
    cv::initUndistortRectifyMap(K, D, R, newK, img_size, CV_32FC1, map1_, map2_);
  } else {
    cv::Mat K32, newK32, R32;
    K.convertTo(K32, CV_32F); newK.convertTo(newK32, CV_32F); R.convertTo(R32, CV_32F);
    cv::fisheye::initUndistortRectifyMap(K32, D, R32, newK32, img_size, CV_32FC1, map1_, map2_);
  }

  RCLCPP_INFO(get_logger(), "Rectification maps built (%s).",
              is_fisheye ? "fisheye/equidistant" : "pinhole/plumb_bob");
}

void CameraPublisher::publishingThreadLoop()
{
  const auto sleep_duration = std::chrono::duration<double>(1.0 / std::max(1.0, fps_));

  while (should_publish_) 
  {
    if (!img_pub_ || !compressed_pub_ || !cinfo_pub_) {
      std::this_thread::sleep_for(sleep_duration);
      continue;
    }

    if (!img_pub_->is_activated() || !compressed_pub_->is_activated() || !cinfo_pub_->is_activated()) {
      std::this_thread::sleep_for(sleep_duration);
      continue;
    }

    auto has_sub = [](const auto & pub) {
      return pub->get_subscription_count() > 0 ||
             pub->get_intra_process_subscription_count() > 0;
    };
    const bool has_raw_sub        = has_sub(img_pub_);
    const bool has_compressed_sub = has_sub(compressed_pub_);
    const bool has_cinfo_sub      = has_sub(cinfo_pub_);

    if (!has_raw_sub && !has_compressed_sub && !has_cinfo_sub) {
      std::this_thread::sleep_for(sleep_duration);
      continue;
    }

    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read frame");
      std::this_thread::sleep_for(sleep_duration);
      continue;
    }

    if (frame.cols != width_ || frame.rows != height_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Driver returned %dx%d, expected %dx%d.", frame.cols, frame.rows, width_, height_);
    }

    if (rectify_ && !map1_.empty()) {
      cv::Mat rectified;
      cv::remap(frame, rectified, map1_, map2_, cv::INTER_LINEAR);
      frame = std::move(rectified);
    }

    const rclcpp::Time stamp = now();
    std_msgs::msg::Header header;
    header.stamp    = stamp;
    header.frame_id = frame_id_;

    if (has_raw_sub) {
      auto msg = std::make_unique<Image>();
      cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*msg);
      img_pub_->publish(std::move(msg));
    }

    if (has_compressed_sub) {
      try {
        std::vector<uint8_t> buf;
        const std::vector<int> encode_params{cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", frame, buf, encode_params);

        auto cmsg = std::make_unique<CompressedImage>();
        cmsg->header = header;
        cmsg->format = "jpeg";
        cmsg->data   = std::move(buf);
        compressed_pub_->publish(std::move(cmsg));
      } catch (const cv::Exception & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to encode/publish JPEG: %s", e.what());
      }
    }

    if (has_cinfo_sub) {
      auto cinfo = std::make_unique<CameraInfo>(curr_cinfo_);
      cinfo->header = header;
      cinfo->width  = static_cast<uint32_t>(frame.cols);
      cinfo->height = static_cast<uint32_t>(frame.rows);
      cinfo_pub_->publish(std::move(cinfo));
    }

    std::this_thread::sleep_for(sleep_duration);
  }
}


} // namespace camera_rospkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera_rospkg::CameraPublisher)
