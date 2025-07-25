#ifndef PHYSICAL_CAMERA_H
#define PHYSICAL_CAMERA_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <camera_rospkg/utils.h>

class CameraPublisher {
public:
    CameraPublisher(ros::NodeHandle& nh);
    void publishImage();
    void publishRecordingStatus();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    cv::VideoCapture cap_;
    cv::Mat frame_, raw_frame_;
    int wb_temp_;
    sensor_msgs::ImagePtr msg_;
    std::string input_;
    std::string topic_name_;
    bool is_wsl2_;
    bool is_display_;
    bool is_recording_started_ = false;
    std::string mp4_output_folder_;
    cv::VideoWriter video_writer_;

    std::string calibration_yaml_path_;
    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Mat map1_, map2_;
    bool is_calibration_enabled_ = false;
    void loadCameraCalibration();

    ros::ServiceServer toggle_recording_srv_;
    bool ToggleRecordingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    ros::ServiceServer enable_camera_srv_;
    bool EnableCameraCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool is_enabled_;

    ros::Publisher recording_status_pub_;
};

#endif // PHYSICAL_CAMERA_H