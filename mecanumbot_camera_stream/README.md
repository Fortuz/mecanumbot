# mecanumbot_camera_stream

ROS 2 package that captures frames from USB (UVC) or CSI/ribbon cameras on the robot's NVIDIA Jetson Orin Nano and publishes `sensor_msgs/msg/Image`.

## Platform

The nodes detect the board from `/proc/device-tree/model` and pick the matching capture path:

- **Jetson Orin Nano (the robot):** CSI capture goes through `nvarguscamerasrc` + `nvvidconv` (JetPack 6 / Argus), and H.264 encoding uses the NVENC element `nvv4l2h264enc`.

**The robot's camera is a USB (UVC) webcam on `/dev/video0`, not a CSI one.** The DeepStream detectors in `mecanumbot_sensorprocess_smart` open the same device with `v4l2src`. So `usb` is the default backend in every config file here and in `camera_compressed.launch.py`. `csi` sends capture through `nvarguscamerasrc`, which cannot open that device: the node logs `Camera open failed` and keeps retrying. Only select `csi` if a ribbon camera is fitted.
- Other boards fall back to their own pipelines (`libcamera` on Raspberry Pi, plain V4L2 elsewhere). These paths are kept for development machines and are not exercised on the robot.

USB (UVC) capture is plain OpenCV/V4L2 and is identical on every platform.

## Nodes

| Executable | Launch file (config) | Default topic |
| --- | --- | --- |
| `camera_image_publisher_node` | `camera_image_publisher.launch.py` (`camera_medium.yaml`) | `/camera/image_raw` |
| `compressed_camera_publisher_node` | `camera_compressed.launch.py` (`camera_compressed.yaml`) | `/camera/image_raw/compressed` |
| `h264_camera_publisher_node` | `camera_h264.launch.py` (`camera_h264.yaml`) | `/camera/image_raw/h264` |

`camera_compressed.launch.py` is the one the rest of the workspace uses (`mecanumbot_bringup/camera.launch.py` includes it, and so does `mecanumbot_autoslam`'s `launch_autoslam.launch.py`). Its `width`/`height`/`fps`/`jpeg_quality`/`camera_backend`/`device`/`topic_name` arguments are applied as overrides on top of the config file, and `publish_fps` is set to `fps`. `camera_image_publisher.launch.py` passes `camera_backend:=auto` by default, which overrides the config's `usb`; `auto` tries USB first and falls back to CSI. The H.264 node has no `camera_backend` parameter.

## Performance Notes

- `use_capture_thread` (default: `true`): runs frame grabbing in a dedicated thread.
- `publish_fps`: publish rate independent from camera read loop.
- USB path uses OpenCV buffer size `1` when supported to reduce frame lag.
- CSI pipeline uses `appsink max-buffers=1 drop=true sync=false` for low latency.

## Quick Start

```bash
colcon build --packages-select mecanumbot_camera_stream
source install/setup.bash
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py
```

## Example Overrides

USB camera:

```bash
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
  camera_backend:=usb \
  device:=/dev/video0
```

CSI/ribbon camera:

```bash
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
  camera_backend:=csi \
  width:=1280 \
  height:=720 \
  fps:=30.0
```

`csi_sensor_id`, `csi_flip_method` and `publish_fps` are node parameters, not launch arguments: passing them to the launch file does nothing. Set them in a `params_file`, or run the node directly.

Higher throughput publish settings:

```bash
ros2 run mecanumbot_camera_stream camera_image_publisher_node --ros-args \
  -p publish_fps:=60.0
```
