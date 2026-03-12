# camera_rospkg

ROS 2 lifecycle camera publisher as a composable node.

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/<ns>/camera_publisher/image_raw` | `sensor_msgs/msg/Image` | BGR8 raw or rectified frame |
| `/<ns>/camera_publisher/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG-compressed frame |
| `/<ns>/camera_publisher/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration info, timestamped with each frame |

Default namespace `<ns>` is `camera_rospkg`.

## Configuration

All node parameters live in [`config/camera_params.yaml`](config/camera_params.yaml):

| Parameter | Default | Description |
|---|---|---|
| `device` | `/dev/video0` | V4L2 device path, index, or video file |
| `frame_id` | `camera_optical_frame` | TF frame stamped on messages |
| `width` | `640` | Capture width (px) |
| `height` | `480` | Capture height (px) |
| `fps` | `30.0` | Capture framerate |
| `rectify` | `true` | Apply undistort/remap using calibration |
| `calibration_url` | `""` | Absolute path to calibration YAML; empty = use package default |
| `autostart` | `true` | *(launch script only)* auto configure + activate after launch |

Camera intrinsics are stored in [`config/calibration.yaml`](config/calibration.yaml).

## Launch

```bash
ros2 launch camera_rospkg camera.launch.py
```

With a custom params file (e.g. when used inside a larger package):

```bash
ros2 launch camera_rospkg camera.launch.py params_file:=/path/to/my_camera.yaml
```

With a custom namespace:

```bash
ros2 launch camera_rospkg camera.launch.py namespace:=front_camera
```

> **Note:** if you change `namespace`, update the top-level key in your params YAML to match: `/<namespace>/camera_publisher`.

## Lifecycle Control

The node autostarts by default (`autostart: true` in `camera_params.yaml`).

```bash
ros2 lifecycle get /camera_rospkg/camera_publisher
ros2 lifecycle set /camera_rospkg/camera_publisher deactivate   # pause publishing
ros2 lifecycle set /camera_rospkg/camera_publisher activate     # resume
ros2 lifecycle set /camera_rospkg/camera_publisher cleanup      # release camera
```
