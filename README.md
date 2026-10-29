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
| `output_width` | `0` | If >0, downscale (INTER_AREA) to this width after rectification; `camera_info` intrinsics are scaled to match |
| `output_height` | `0` | Output height, set together with `output_width` |
| `v4l2_controls` | `[]` | Camera hardware settings as `"name=value"` strings, applied in order (see below) |

### Camera hardware controls (`v4l2_controls`)

Saturation, contrast, white balance, exposure etc. are set on the camera itself, so they cost no CPU and are applied
before MJPG compression. Names and ranges come from the device, so any V4L2 camera works without code changes:

```bash
v4l2-ctl -d /dev/video0 --list-ctrls      # names, ranges and defaults for this camera
```

```yaml
v4l2_controls:
  - "white_balance_automatic=0"           # turn auto modes off before the manual values they unlock
  - "white_balance_temperature=5000"
  - "saturation=72"
```

Entries are applied in list order on configure. Unknown names, bad values and out-of-range values are skipped with a
warning; an unknown name also logs every control the camera supports. To tune live while the camera is running:

```bash
ros2 param set /<ns>/perception/camera_publisher_front v4l2_controls "['saturation=80', 'contrast=40']"
```

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
