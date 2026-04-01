# mecanumbot_camera_stream

ROS 2 package that captures frames from USB (UVC) or CSI/ribbon cameras on NVIDIA Orin Nano and publishes `sensor_msgs/msg/Image`.

## Node

- Executable: `camera_image_publisher_node`
- Default topic: `/camera/image_raw`

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
  csi_sensor_id:=0 \
  width:=1280 \
  height:=720 \
  fps:=30
```

Higher throughput publish settings:

```bash
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
  publish_fps:=60.0
```
