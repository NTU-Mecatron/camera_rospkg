#include "camera_rospkg/image_decompressor.hpp"
#include <sensor_msgs/image_encodings.hpp>

namespace
{
constexpr char compressedImageTopic[] = "image/compressed";
constexpr char rawImageTopic[] = "image/raw";
} // Anonymous namespace

namespace camera_rospkg {

// Component node to decompress image/compressed and republish as image/raw.
ImageDecompressor::ImageDecompressor(const rclcpp::NodeOptions & options)
: Node("image_decompressor", options)
{
  encoding_ = declare_parameter<std::string>("output_encoding", sensor_msgs::image_encodings::BGR8);

  // Use SensorDataQoS to match typical camera publisher settings
  compressed_sub_ = create_subscription<CompressedImage>(
    compressedImageTopic, rclcpp::SensorDataQoS(),
    [this](CompressedImage::ConstSharedPtr msg) { onImage(msg); });

  image_pub_ = create_publisher<Image>(rawImageTopic, rclcpp::QoS(10).reliable());

  RCLCPP_INFO(get_logger(), "Image Decompressor initialized: sub=%s, pub=%s", compressedImageTopic, rawImageTopic);
}

void ImageDecompressor::onImage(CompressedImage::ConstSharedPtr msg)
{
  try {
    // cv_bridge handles decoding (imdecode) for CompressedImage types
    auto cv_ptr = cv_bridge::toCvCopy(*msg, encoding_);
    
    if (!cv_ptr) return;

    // Prepare output message as a unique_ptr to facilitate zero-copy intra-process transfer
    auto out_msg = std::make_unique<Image>();
    cv_ptr->toImageMsg(*out_msg);
    
    // Metadata propagation
    out_msg->header = msg->header;

    image_pub_->publish(std::move(out_msg));
    
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, 
      "Decompression error: %s", e.what());
  }
}

} // namespace camera_rospkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera_rospkg::ImageDecompressor)