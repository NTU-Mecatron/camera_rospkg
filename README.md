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

Camera intrinsics are stored in [`config/calibration.yaml`](config/calibration.yaml).

### Camera controls

Optional integer params, applied through `cv::VideoCapture::set` when the camera is configured. Leave one out to keep the camera's own setting. Values are raw driver values: names, ranges and defaults come from `v4l2-ctl -d <device> --list-ctrls`. The log shows the value the camera reports back for each one.

| Parameter | OpenCV property | Note |
|---|---|---|
| `white_balance_automatic` | `CAP_PROP_AUTO_WB` | Set 0 for `white_balance_temperature` to take effect |
| `white_balance_temperature` | `CAP_PROP_WB_TEMPERATURE` | |
| `auto_exposure` | `CAP_PROP_AUTO_EXPOSURE` | Menu value; manual mode needed for `exposure_time_absolute` |
| `exposure_time_absolute` | `CAP_PROP_EXPOSURE` | |
| `gain` | `CAP_PROP_GAIN` | |
| `brightness` | `CAP_PROP_BRIGHTNESS` | |
| `contrast` | `CAP_PROP_CONTRAST` | |
| `saturation` | `CAP_PROP_SATURATION` | |

Changes take effect on the next configure (`ros2 param set`, then deactivate/cleanup/configure/activate).

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
