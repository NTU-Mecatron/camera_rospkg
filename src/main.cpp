#include <camera_rospkg/camera_publisher.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh("~");
    ros::Time last_status_pub_time = ros::Time::now();

    CameraPublisher camera_publisher(nh);

    int frame_rate;
    nh.param<int>("frame_rate", frame_rate, 30);
    ros::Rate loop_rate(frame_rate);

    while (ros::ok()) 
    {
        camera_publisher.publishImage();
        
        // Publish the recording status in 1 hz
        if (ros::Time::now() - last_status_pub_time > ros::Duration(1.0)) {
            camera_publisher.publishRecordingStatus();
            last_status_pub_time = ros::Time::now();
        }

        loop_rate.sleep();
    }

    return 0;
}