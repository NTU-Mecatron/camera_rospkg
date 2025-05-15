#ifndef PHYSICAL_CAMERA_H
#define PHYSICAL_CAMERA_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <camera_rospkg/utils.h>
#include <camera_rospkg/StartRecording.h>
#include <camera_rospkg/StopRecording.h>

class CameraPublisher {
public:
    CameraPublisher(ros::NodeHandle& nh);
    void publishImage();

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

    ros::ServiceServer start_recording_srv_;
    ros::ServiceServer stop_recording_srv_;
    bool startRecordingCallback(camera_rospkg::StartRecording::Request &req, camera_rospkg::StartRecording::Response &res);
    bool stopRecordingCallback(camera_rospkg::StopRecording::Request &req, camera_rospkg::StopRecording::Response &res);
};

#endif // PHYSICAL_CAMERA_H
