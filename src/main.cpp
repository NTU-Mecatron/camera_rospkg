#include <camera_rospkg/camera_publisher.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh("~");

    CameraPublisher camera_publisher(nh);

    int frame_rate;
    nh.param<int>("frame_rate", frame_rate, 30);
    ros::Rate loop_rate(frame_rate);

    while (ros::ok()) 
    {
        camera_publisher.publishImage();
        loop_rate.sleep();
    }

    return 0;
}