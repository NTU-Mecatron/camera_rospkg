#include <camera_rospkg/camera_publisher.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh("~");

    CameraPublisher camera_publisher(nh);

    while (ros::ok()) {
        camera_publisher.publishImage();
        ros::spinOnce();
        cv::waitKey(5);
        // loop_rate.sleep();
    }

    return 0;
}