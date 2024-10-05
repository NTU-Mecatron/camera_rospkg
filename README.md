# Camera Publisher Package

## Overview

The `camera_rospkg` package provides tools for publishing images from a camera device to a ROS topic. It also works with saved videos and allows for displaying the frames on display (if display is available).

> Note: you can launch *multiple instances* of the package, each with different node names and camera devices. This allows for streaming images from multiple cameras to different topics. To do this, you just need to instantiate the 'camera_publisher' executable with different node names, camera devices and topic names.

## Installation

1. Clone the repository:
    ```
    cd /path/to/catkin_ws/src
    git clone https://github.com/NTU-Mecatron/camera_rospkg.git camera_rospkg
    ```

2. Build the package:
    ```
    cd ..
    catkin_make
    ```

3. Open rviz:
    ```
    source devel/setup.bash
    roslaunch camera_rospkg rviz.launch
    ```

4. Open a new terminal and run the camera publisher:
    ```
    roslaunch camera_rospkg camera.launch
    ```

## Explanation

#### Launch file

The launch file contains the following parameters:
- `input` (string): camera device such as `/dev/video0` or video file path.
- `topic_name` (string): the topic name to publish the images to.
- `is_wsl2` (bool): whether to MJPG encoding. Discourage to use this when not working with WSL2. Default: false.
- `is_display` (bool): whether to display the images on screen. Default: false.

You may also notice that there is also the parameter list `disable_pub_plugins` which disables the plugins such as `compressed`, `compressedDepth` and `theora` publishers. These plugins are not required and can cause additional bugs when not used.

#### Published Topics

- `/sensor/camera` (`sensor_msgs/Image`): The raw image topic from the camera.

**Warning**: you are discouraged to use this topic name directly in your other packages. Instead, you are encouraged to get this parameter from ros parameter server as follows:

```cpp
// Replace ${THIS_NODE_NAME} with the name of the node you are using.
// In this case, it will be camera_publisher.
nh.getParam("/${THIS_NODE_NAME}/topic_name", camera_topic);
```

## Known limitations

The camera recording feature (saving to mp4) is not fully implemented and will not work. It is work in progress.
