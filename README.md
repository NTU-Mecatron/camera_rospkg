# camera_rospkg

ROS 2 lifecycle camera publisher as a composable node.

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/<ns>/image/raw` | `sensor_msgs/msg/Image` | BGR8 raw or rectified frame |
| `/<ns>/image/raw/compressed` | `sensor_msgs/msg/CompressedImage` | `image_transport` compressed output for the configured base topic |
| `/<ns>/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration info, timestamped with each frame |

> Note: raw + compressed publication is now handled through `image_transport` on a temporary non-lifecycle node created with the same node options, so intra-process communication remains enabled for pointer-based raw publication.

> The image transport base topic is fixed to `image/raw`, and only the raw + compressed publisher plugins are enabled.

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
