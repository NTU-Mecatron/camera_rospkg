#include <camera_rospkg/camera_publisher.h>

CameraPublisher::CameraPublisher(ros::NodeHandle& nh) : nh_(nh), it_(nh_) 
{
    // Get the topic name from the parameter server
    nh_.param<std::string>("input", input_, "/dev/video0");
    nh_.param<std::string>("topic_name", topic_name_, "/sensor/camera");
    nh_.param<bool>("is_wsl2", is_wsl2_, false);
    nh_.param<bool>("is_display", is_display_, false);
    nh_.param<std::string>("mp4_output_folder", mp4_output_folder_, "");

    std::string mp4_file_name = "";

    if (mp4_output_folder_ == "")
        ROS_INFO("MP4 output path not set.");
    else {
        // Remove trailing '/' if it exists
        if (mp4_output_folder_.back() == '/') {
            mp4_output_folder_.pop_back();
        }

        mp4_file_name = generateMP4FileName();
        mp4_file_name = mp4_output_folder_ + "/" + mp4_file_name;
        ROS_WARN("Saving MP4 video to: %s", mp4_file_name.c_str());
    }

    // Initialize the publisher
    image_pub_ = it_.advertise(topic_name_, 1);
    ROS_INFO("Publishing camera images to topic: %s", topic_name_.c_str());

    // Open the camera in WSL2
    if (is_wsl2_) 
    {
        cap_.open(input_, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        ROS_WARN("WSL2 mode. MJPG codec enabled!");
    }
    else 
    {
        cap_.open(input_);
        ROS_WARN("Native mode. MJPG codec disabled.");
    }

    if (!cap_.isOpened()) 
    {
        ROS_ERROR("Failed to open camera");
        ros::shutdown();
    }
    else 
    {
        ROS_INFO("Camera opened successfully");
    }
    
    // Initialize the VideoWriter if the MP4 output path is set
    if (!mp4_file_name.empty()) {
        int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        video_writer_.open(mp4_file_name, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(frame_width, frame_height), true);

        if (!video_writer_.isOpened()) {
            ROS_ERROR("Failed to open video writer");
            ros::shutdown();
        }
    }
}

void CameraPublisher::publishImage() 
{
    cap_ >> frame_; // Capture a frame

    if (is_display_) 
        cv::imshow("frame", frame_); // Display the frame

    if (!frame_.empty()) 
    {
        // Convert the frame to a ROS image message
        msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
        image_pub_.publish(msg_);
    }

    if (video_writer_.isOpened()) 
        video_writer_ << frame_; // Write the frame to the video
}
