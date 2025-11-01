# camera_rospkg

This ROS 2 package provides a node to capture video from a standard USB camera (e.g., V4L2 devices on Linux). It publishes the video as `sensor_msgs/msg/Image` topics and `sensor_msgs/msg/CameraInfo`, with support for camera calibration, image rectification, and compressed video transport.

## Features

-   Captures from USB cameras specified by device path (e.g., `/dev/video0`) or index.
-   Publishes raw images via `image_transport`, allowing for various transport plugins.
-   Publishes `sensor_msgs/msg/CameraInfo` synchronized with the image stream.
-   Loads camera calibration data from a `.yaml` file using `camera_info_manager`.
-   Performs image rectification for both standard (`plumb_bob`) and fisheye (`equidistant`) lens models.
-   Configurable via ROS parameters for resolution, frame rate, pixel format, and more.

## Dependencies

To use this package, you need to install the following ROS 2 and system dependencies:

```bash
sudo apt update
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-cv-bridge
sudo apt install libopencv-dev
```

## Building

To build the package, navigate to your workspace root and run `colcon build`:

```bash
# From your workspace root (e.g., ~/auv_ws_bk)
colcon build --packages-select camera_rospkg
```

## Usage

Source your workspace and run the provided launch file. You can specify a namespace for the node.

```bash
# Source the workspace
source install/setup.bash

# Launch the camera node
ros2 launch camera_rospkg camera.launch.py namespace:=my_camera
```

The node will start publishing topics under the specified namespace (e.g., `/my_camera/image_raw` and `/my_camera/camera_info`).

### Parameters

The node's behavior can be configured through parameters in the [`launch/camera.launch.py`](launch/camera.launch.py) file or via the command line. Key parameters include:

-   `device`: Camera device path (e.g., `'/dev/video0'`).
-   `width`, `height`: Capture resolution.
-   `fps`: Capture frame rate.
-   `rectify`: Set to `True` to enable image rectification.
-   `calibration_url`: Path to the camera calibration file.
-   `pixel_format`: The desired pixel format (e.g., `'MJPG'`).

## Topics Published

-   `image_raw` (`sensor_msgs/msg/Image`): The raw or rectified image stream.
-   `camera_info` (`sensor_msgs/msg/CameraInfo`): The corresponding camera calibration information.
-   `image_raw/compressed` (`sensor_msgs/msg/CompressedVideo`): H.264 compressed video stream 