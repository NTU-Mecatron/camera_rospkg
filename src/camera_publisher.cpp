#include <camera_rospkg/camera_publisher.h>

CameraPublisher::CameraPublisher(ros::NodeHandle& nh) : nh_(nh), it_(nh_) 
{
    // Get the topic name from the parameter server
    nh_.getParam("input", input_);
    nh_.getParam("topic_name", topic_name_);
    nh_.getParam("is_wsl2", is_wsl2_);
    nh_.getParam("is_display", is_display_);
    nh_.getParam("mp4_output_path", mp4_output_path_);

    if (mp4_output_path_ == "")
        ROS_INFO("MP4 output path not set.");
    else
        ROS_INFO("Saving MP4 video to: %s", mp4_output_path_.c_str());

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
    if (!mp4_output_path_.empty()) {
        int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        video_writer_.open(mp4_output_path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(frame_width, frame_height), true);

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
