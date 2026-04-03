# camera_rospkg

ROS 2 lifecycle camera publisher as a composable node.

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/<ns>/image/raw` | `sensor_msgs/msg/Image` | BGR8 raw or rectified frame |
| `/<ns>/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG-compressed frame |
| `/<ns>/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration info, timestamped with each frame |

> Note: we do not use image_transport so that we can utilise zero-copy intra-process communication.

## Configuration

All node parameters live in [`config/camera_params.yaml`](config/camera_params.yaml):

| Parameter | Default | Description |
|---|---|---|
| `device` | `/dev/video0` | V4L2 device path, index, or video file |
| `frame_id` | `camera_link` | TF frame stamped on messages |
| `width` | `640` | Capture width (px) |
| `height` | `480` | Capture height (px) |
| `fps` | `30.0` | Capture framerate |
| `rectify` | `true` | Apply undistort/remap using calibration |

Camera intrinsics are stored in [`config/calibration.yaml`](config/calibration.yaml).

## Launch

With a custom namespace:

```bash
ros2 launch camera_rospkg camera.launch.py namespace:=front_camera
```

## Lifecycle Control

The node configures and activates by default when launched.

```bash
ros2 lifecycle get /camera/camera_publisher
ros2 lifecycle set /camera/camera_publisher deactivate   # pause publishing
ros2 lifecycle set /camera/camera_publisher activate     # resume
ros2 lifecycle set /camera/camera_publisher cleanup      # release camera
```
