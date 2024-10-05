#ifndef PHYSICAL_CAMERA_H
#define PHYSICAL_CAMERA_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

class CameraPublisher {
public:
    CameraPublisher(ros::NodeHandle& nh);
    void publishImage();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    cv::VideoCapture cap_;
    cv::Mat frame_;
    sensor_msgs::ImagePtr msg_;
    std::string input_ = "/dev/video0";
    std::string topic_name_ = "/sensor/camera";
    bool is_wsl2_ = false;
    bool is_display_ = false;
    std::string mp4_output_path_ = "";
    cv::VideoWriter video_writer_;
};

#endif // PHYSICAL_CAMERA_H