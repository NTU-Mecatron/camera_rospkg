# Camera Publisher Package

**Table of Contents**
- [Overview](#overview)
- [Getting started](#getting-started)
- [Explanation](#explanation)
  - [Launch file](#launch-file)
  - [Published Topics](#published-topics)
- [Known limitations](#known-limitations)
- [Common issues](#common-issues)
  - ["cv_bridge" cannot find OpenCV](#cv_bridge-cannot-find-opencv)
  - [Could not find module FindOpenCV.cmake](#could-not-find-module-findopencvcmake)

## Overview

The `camera_rospkg` package provides tools for publishing images from a camera device to a ROS topic. It also works with saved videos and allows for displaying the frames on display (if display is available).

> Note: you can launch *multiple instances* of the package, each with different node names and camera devices. This allows for streaming images from multiple cameras to different topics. To do this, you just need to instantiate the 'camera_publisher' executable with different node names, camera devices and topic names.

## Getting started

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

You may also notice that there is also the parameter list `disable_pub_plugins` which disables the plugins such as `compressedDepth` and `theora` publishers. These plugins are not required and can cause additional bugs when not used.

In the launch file, there is also a section to enable rosbag recording. Just uncomment this block of code and it will work!

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

## Common issues

#### "cv_bridge" cannot find OpenCV

If you encounter the following error:

```
CMake Error at /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake:113 (message):
  Project 'cv_bridge' specifies '/usr/include/opencv' as an include dir,
  which is not found.  It does neither exist as an absolute directory nor in
  '${{prefix}}//usr/include/opencv'.  Check the issue tracker
  'https://github.com/ros-perception/vision_opencv/issues' and consider
  creating a ticket if the problem has not been reported yet.
Call Stack (most recent call first):
  /opt/ros/melodic/share/catkin/cmake/catkinConfig.cmake:76 (find_package)
  vision_to_mavros/CMakeLists.txt:10 (find_package)
```

Open the folowing file:

```
sudo gedit /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake
```

Change this line:

```
set(_include_dirs "include;/usr/include;/usr/include/opencv")
```

To:

```
set(_include_dirs "include;/usr/include;/usr/include/opencv4")
```

Then, rebuild the package. Original solution found [here](https://github.com/ros-perception/vision_opencv/issues/345).

#### Could not find module FindOpenCV.cmake

Error log:

```
CMake Error at CMakeLists.txt:15 (find_package):
  Could not find module FindOpenCV.cmake or a configuration file for package
  OpenCV.

  Adjust CMAKE_MODULE_PATH to find FindOpenCV.cmake or set OpenCV_DIR to the
  directory containing a CMake configuration file for OpenCV.  The file will
  have one of the following names:

    OpenCVConfig.cmake
    opencv-config.cmake
```

You can set the environment variable `OpenCV_DIR` to the path of opencv (containing the OpenCVConfig.cmake file) like this:

```bashrc
vim ~/.bashrc

# Add the following line to the end of the file
export OpenCV_DIR=/usr/share/opencv4
# Save and exit the file by typing :wq and Enter
```

Then, source the file:

```bashrc
source ~/.bashrc
```

Then, rebuild the package. Original solution found [here](https://stackoverflow.com/questions/8711109/could-not-find-module-findopencv-cmake-error-in-configuration-process).