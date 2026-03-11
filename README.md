# camera_rospkg

`camera_rospkg` provides a ROS 2 lifecycle camera publisher built as a reusable component and launched in a `ComposableNodeContainer`.

## Features

- Lifecycle-managed `image_raw`, `image_raw/compressed`, and `camera_info` publishers.
- Standard lifecycle behavior: `configure` opens the device, `activate` starts publishing, `deactivate` pauses publishing, `cleanup` releases camera and calibration resources.
- `unique_ptr` publishing for raw images, JPEG-compressed images, and camera info messages.
- Subscriber-aware capture loop that skips frame acquisition when no normal or intra-process subscribers are present.
- Rectification support from a calibration YAML file.

## Runtime Mode

- `ros2 launch camera_rospkg camera.launch.py`
  Runs the camera as a composable node inside a single-threaded component container with `use_intra_process_comms:=true` by default.

The launch path autostarts internally. The node configures and activates itself once the executor starts spinning.

## Launch Scripts

Use this when composing the camera with other nodes in the same process:

```bash
ros2 launch camera_rospkg camera.launch.py
```

Example with a custom device and namespace:

```bash
ros2 launch camera_rospkg camera.launch.py \
  namespace:=front_camera \
  device:=/dev/video0
```

## Common Launch Arguments

- `namespace`: Namespace for the camera topics and node.
- `device`: Camera device path or numeric index.
- `width`, `height`, `fps`: Requested capture settings.
- `frame_id`: Frame ID stamped into outgoing messages.
- `rectify`: Enable or disable rectification.
- `calibration_url`: Path to the calibration YAML file.
The composable launch also accepts `use_intra_process_comms`, defaulting to `true`.

## Notes

- Zero-copy is best-effort. The `unique_ptr` publish path and `use_intra_process_comms:=true` help local composed pipelines, but actual zero-copy still depends on the subscriber path and middleware capabilities.
- `image_raw/compressed` is published as `sensor_msgs/msg/CompressedImage` encoded with OpenCV JPEG compression.
- After startup, the node still supports normal lifecycle transitions such as `deactivate`, `cleanup`, and `activate` if a higher-level system wants to manage it later.
