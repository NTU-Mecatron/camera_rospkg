#include "camera_rospkg/video_saver.hpp"

#include <algorithm>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

namespace camera_rospkg {

VideoSaver::VideoSaver(const rclcpp::NodeOptions & options): rclcpp::Node("video_saver", options){
    
    // Declare parameters
    fps_             = declare_parameter<double>("fps", 30.0);
    video_codec_     = declare_parameter<std::string>("video_codec", "mp4v");
    output_folder_   = declare_parameter<std::string>("output_folder", "/video_folder");
    width_           = declare_parameter<int>("width", 640);
    height_          = declare_parameter<int>("height", 480);

    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << output_folder_ << "/video_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".mp4";
    std::string filename = ss.str();

    // Initialize VideoWriter
    int fourcc = cv::VideoWriter::fourcc(video_codec_[0], video_codec_[1], 
                                          video_codec_[2], video_codec_[3]);
    video_writer_.open(filename, fourcc, fps_, cv::Size(width_, height_), true);

    if (!video_writer_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open video writer: %s", filename.c_str());
        throw std::runtime_error("VideoWriter initialization failed");
    }

    setupSubscriber(); 
    RCLCPP_INFO(get_logger(), "video_saver started");

}

void VideoSaver::setupSubscriber() {
    // Create a subscriber for the compressed video feed
    video_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/camera_rospkg/image_raw/compressed", 10,
        std::bind(&VideoSaver::videoCallback, this, std::placeholders::_1));
}

void VideoSaver::videoCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        // Decode compressed image back to cv::Mat format
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        
        if (!frame.empty() && video_writer_.isOpened()) {
            video_writer_.write(frame);
        }
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error decoding compressed image: %s", e.what());
    }
}

}
