# mecanumbot_camera_stream

ROS 2 package that captures frames from USB (UVC) or CSI/ribbon cameras on NVIDIA Orin Nano and publishes `sensor_msgs/msg/Image`.

## Node

- Executable: `camera_image_publisher_node`
- Default topic: `/camera/image_raw`

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
