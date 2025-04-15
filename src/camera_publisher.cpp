#include <camera_rospkg/camera_publisher.h>

CameraPublisher::CameraPublisher(ros::NodeHandle& nh) : nh_(nh), it_(nh_) 
{
    // Get the topic name from the parameter server
    nh_.param<std::string>("input", input_, "/dev/video0");
    nh_.param<std::string>("topic_name", topic_name_, "/sensor/camera");
    nh_.param<bool>("is_wsl2", is_wsl2_, false);
    nh_.param<bool>("is_display", is_display_, false);
    nh_.param<std::string>("mp4_output_folder", mp4_output_folder_, "");
    nh_.param<std::string>("calibration_yaml_path", calibration_yaml_path_, "");
    nh_.param<int>("whitebalance_temperature", wb_temp_, -1);

    // Initialize the publisher
    image_pub_ = it_.advertise(topic_name_, 1);
    ROS_INFO("Publishing camera images to topic: %s", topic_name_.c_str());

    // Initialize the service servers
    on_off_recording_srv_ = nh_.advertiseService("on_off_recording", &CameraPublisher::OnOffRecordingCallback, this);

    // Initialize status publisher, queue size 1 for low rate update (1 hz)
    recording_status_pub_ = nh_.advertise<std_msgs::Bool>("recording_status", 1);

    // Open the camera in WSL2
    if (is_wsl2_) 
    {
        cap_.open(input_, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        ROS_WARN("WSL2 mode. Are you sure?");
    }
    else 
        cap_.open(input_);

    if (!cap_.isOpened()) 
    {
        ROS_ERROR("Failed to open camera");
        ros::shutdown();
    }
    else 
        ROS_INFO("Camera opened successfully");

    // Load the camera calibration parameters
    if (calibration_yaml_path_ != "") 
        loadCameraCalibration();
    else 
        ROS_WARN("Camera calibration file NOT provided!");

    // Whitebalance the camera
    if (wb_temp_ != -1) 
    {
        cap_.set(cv::CAP_PROP_AUTO_WB, 0);
        cap_.set(cv::CAP_PROP_WB_TEMPERATURE, wb_temp_);
        ROS_INFO("Whitebalance set to: %d", wb_temp_);
    }
    else cap_.set(cv::CAP_PROP_AUTO_WB, 1);

    if (mp4_output_folder_ == "")
        ROS_WARN("MP4 recording is NOT enabled!");
    else if (mp4_output_folder_.back() == '/')
        mp4_output_folder_.pop_back();
}

void CameraPublisher::publishImage() 
{
    if (is_calibration_enabled_) {
        cap_ >> raw_frame_; // Capture a frame
        cv::remap(raw_frame_, frame_, map1_, map2_, cv::INTER_LINEAR); // Undistort the frame
    }
    else cap_ >> frame_; // Capture a frame

    if (is_display_) {
        cv::imshow("frame", frame_); // Display the frame
        cv::waitKey(10);
    }

    if (!frame_.empty()) 
    {
        // Convert the frame to a ROS image message
        msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
        image_pub_.publish(msg_);
    }

    if (video_writer_.isOpened()) 
        video_writer_ << frame_; // Write the frame to the video

    // Need to spin once to let the service servers work
    ros::spinOnce();
}

bool CameraPublisher::OnOffRecordingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    if (mp4_output_folder_ == "") {
        ROS_ERROR("MP4 recording is NOT enabled! Please set the mp4_output_folder parameter.");
        res.success = false;
        res.message = "MP4 recording is NOT enabled! Please set the mp4_output_folder parameter.";
        return true;
    }
    else if (is_recording_started_) {
        video_writer_.release();
        is_recording_started_ = false;
        ROS_WARN("Recording stopped!");
        res.success = true;
        res.message = "Recording off";
        return true;
    }

    std::string mp4_file_name = "";
    mp4_file_name = generateMP4FileName();
    mp4_file_name = mp4_output_folder_ + "/" + mp4_file_name;
    ROS_WARN("Saving MP4 video to: %s", mp4_file_name.c_str());

    int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    video_writer_.open(mp4_file_name, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(frame_width, frame_height), true);

    if (!video_writer_.isOpened()) {
        ROS_ERROR("Failed to open video writer");
        res.success = false;
        res.message = "Failed to open video writer";
    } else {
        is_recording_started_ = true;
        ROS_WARN("Recording started!");
        res.success = true;
        res.message = "Recording on";
    }
    return true;
}

void CameraPublisher::loadCameraCalibration() 
{
    // Load the camera calibration parameters from the YAML file
    cv::FileStorage fs(calibration_yaml_path_, cv::FileStorage::READ);
    if (!fs.isOpened()) 
    {
        ROS_ERROR("Failed to open camera calibration file: %s", calibration_yaml_path_.c_str());
        ros::shutdown();
    }

    fs["Camera_Matrix"] >> camera_matrix_;
    fs["Distortion_Coefficients"] >> dist_coeffs_;
    fs.release();

    // Check the dimensions of the loaded matrices
    if (camera_matrix_.rows != 3 || camera_matrix_.cols != 3) {
        ROS_ERROR("Invalid camera matrix dimensions!");
    }

    if (dist_coeffs_.rows != 5 || dist_coeffs_.cols != 1) {
        ROS_ERROR("Invalid distortion coefficients dimensions!");
    }

    cv::Size imageSize = cv::Size(static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)), static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));

    cv::initUndistortRectifyMap(
                camera_matrix_, dist_coeffs_, cv::Mat(),
                cv::getOptimalNewCameraMatrix(camera_matrix_, dist_coeffs_, imageSize, 0, imageSize, 0), imageSize,
                CV_16SC2, map1_, map2_);

    ROS_WARN("Camera calibration enabled!");
    is_calibration_enabled_ = true;
}

void CameraPublisher::publishRecordingStatus() {
    // Publish the recording status    
    std_msgs::Bool status_msg;
    status_msg.data = is_recording_started_;
    recording_status_pub_.publish(status_msg);
}