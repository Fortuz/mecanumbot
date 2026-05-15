# Camera Stream Optimization Guide

This package provides optimized camera streaming for the Mecanumbot with multiple configurations for different bandwidth and quality requirements.

## 📊 Bandwidth Comparison

| Configuration | Resolution | FPS | Format | Bandwidth | CPU Usage | Use Case |
|--------------|-----------|-----|---------|-----------|-----------|----------|
| **Raw Low** | 320x240 | 15 | BGR8 | ~3.5 MB/s | Low | Basic navigation |
| **Raw Medium** | 640x480 | 15 | BGR8 | ~13.5 MB/s | Low | Standard operations |
| **Raw High** | 1280x720 | 30 | BGR8 | ~26 MB/s | Medium | High quality |
| **JPEG Compressed** | 640x480 | 15 | JPEG | ~0.5 MB/s | Medium | **Recommended for most use cases** |
| **JPEG HQ** | 1280x720 | 15 | JPEG | ~1-2 MB/s | Medium | Remote viewing |
| **H.264** | 640x480 | 15 | H.264 | ~0.25 MB/s | Low* | Ultra-low bandwidth |
| **H.264 HQ** | 1280x720 | 30 | H.264 | ~0.5 MB/s | Low* | Recording/streaming |

*H.264 requires hardware encoder for low CPU usage

## 🚀 Quick Start

### Option 1: JPEG Compressed (Recommended)
**Best balance of quality, bandwidth, and compatibility**

```bash
# Launch with default 640x480@15fps, JPEG quality 80
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py

# Custom quality (0-100, higher is better)
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py jpeg_quality:=90

# High quality 720p
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_compressed_hq.yaml
```

**Subscribe to:**
```bash
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw/compressed
# OR for compressed subscriber:
ros2 topic echo /camera/image_raw/compressed
```

### Option 2: H.264 Hardware Encoding (Ultra-low bandwidth)
**Best for remote streaming, multi-camera setups, or cellular connections**

```bash
# Auto-detect hardware encoder (NVENC/Jetson, VAAPI, OMX, or x264 fallback)
ros2 launch mecanumbot_camera_stream camera_h264.launch.py

# High quality 720p@30fps
ros2 launch mecanumbot_camera_stream camera_h264.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_h264_hq.yaml

# Specify hardware encoder
ros2 launch mecanumbot_camera_stream camera_h264.launch.py hardware_encoder:=nvenc

# Custom bitrate (bits per second)
ros2 launch mecanumbot_camera_stream camera_h264.launch.py bitrate:=4000000
```

**Subscribe to:**
```bash
# Requires H.264 decoder
ros2 topic echo /camera/image_raw/h264
```

### Option 3: Raw Image with Profiles
**Use when you need uncompressed images for processing**

```bash
# Medium quality (640x480@15fps) - default
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py

# Low bandwidth (320x240@15fps)
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_low_bandwidth.yaml

# High quality (1280x720@30fps) - original config
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_high.yaml
```

## 📁 Available Configuration Files

### Raw Images
- `camera_low_bandwidth.yaml` - 320x240@15fps - Battery saving, basic navigation
- `camera_medium.yaml` - 640x480@15fps - Standard robotics operations ⭐ **Recommended raw**
- `camera_high.yaml` - 1280x720@30fps - High quality recording
- `camera_stream.yaml` - Original 1280x720@30fps config (legacy)

### Compressed Images
- `camera_compressed.yaml` - 640x480@15fps JPEG 80% - General purpose ⭐ **Recommended overall**
- `camera_compressed_hq.yaml` - 1280x720@15fps JPEG 85% - High quality with compression

### H.264 Hardware Encoding
- `camera_h264.yaml` - 640x480@15fps 2Mbps - Ultra-low bandwidth
- `camera_h264_hq.yaml` - 1280x720@30fps 4Mbps - High quality streaming

## 🔧 Launch Parameters

### All Nodes
- `device` - Camera device path (default: `/dev/video0`)
- `width` - Image width in pixels
- `height` - Image height in pixels
- `fps` - Frames per second
- `topic_name` - Output topic name
- `frame_id` - TF frame ID
- `params_file` - Path to config YAML file

### Raw Image Node Only
- `camera_backend` - Backend: `auto`, `usb`, or `csi`
- `encoding` - Image encoding: `bgr8`, `rgb8`, `mono8`
- `publish_fps` - Publish rate (can be lower than capture fps)

### Compressed Node Only
- `format` - Compression format: `jpeg` or `png`
- `jpeg_quality` - JPEG quality 0-100 (default: 80)
- `png_level` - PNG compression 0-9 (default: 3)

### H.264 Node Only
- `bitrate` - H.264 bitrate in bps (default: 2000000 = 2Mbps)
- `hardware_encoder` - Encoder: `auto`, `nvenc`, `vaapi`, `omx`, `x264`
- `custom_pipeline` - Custom GStreamer pipeline (overrides auto-detection)

## 🎯 Which Configuration Should I Use?

### For Local Processing (Computer Vision, SLAM, Navigation)
- Use **Raw Medium** (640x480@15fps) or **Raw Low** for battery saving
- Uncompressed images are faster for CV algorithms to process
- Lower resolution reduces CPU load in your processing pipeline

### For Remote Viewing/Monitoring
- Use **JPEG Compressed** (default) - best compatibility and quality
- Use **H.264** for cellular connections or when bandwidth is critical
- JPEG works with standard ROS 2 image viewers

### For Recording
- Use **H.264 HQ** - smallest file size with excellent quality
- Use **JPEG HQ** if you need frame-by-frame access

### For Multi-Camera Systems
- Use **H.264** - hardware encoding offloads CPU
- Can run multiple cameras without overloading the system

### For Battery-Constrained Operation
- Use **Raw Low** (320x240@15fps) - minimal processing
- Or **JPEG Compressed** with lower quality setting

## 🛠️ Hardware Requirements

### JPEG Compression
- ✅ Works on all systems
- CPU usage: Medium (software encoding)
- Requires: OpenCV

### H.264 Hardware Encoding
- ✅ **Jetson (Orin, Xavier, Nano)**: Uses NVENC hardware encoder (very efficient)
- ✅ **Intel CPUs with iGPU**: Uses VAAPI hardware encoder
- ✅ **Raspberry Pi**: Uses OMX hardware encoder
- ⚠️ **Fallback**: x264 software encoder (high CPU usage)

Check if hardware encoder is available:
```bash
# For NVENC (Jetson)
gst-inspect-1.0 nvv4l2h264enc

# For VAAPI (Intel)
gst-inspect-1.0 vaapih264enc

# For OMX (Raspberry Pi)
gst-inspect-1.0 omxh264enc
```

## 🔍 Viewing Compressed Images

### JPEG Compressed
```bash
# Using image_tools
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw/compressed

# Using rqt_image_view
ros2 run rqt_image_view rqt_image_view /camera/image_raw/compressed

# Using compressed_image_transport (republish as raw)
ros2 run image_transport republish compressed raw \
    --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_decompressed
```

### H.264
H.264 requires a custom decoder. You can:
1. Use a GStreamer-based viewer
2. Record and playback with video players
3. Build a custom subscriber node with H.264 decoder

## 📈 Performance Monitoring

Check actual bandwidth usage:
```bash
# Monitor topic bandwidth
ros2 topic bw /camera/image_raw/compressed

# Monitor publish rate
ros2 topic hz /camera/image_raw/compressed

# View topic info
ros2 topic info /camera/image_raw/compressed -v
```

Check CPU usage:
```bash
top -p $(pgrep -f camera_publisher_node)
```

## 🔄 Migrating from Old Configuration

If you're currently using the original high-bandwidth config:

**Before (1280x720@30fps raw = ~26 MB/s):**
```bash
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py
```

**After (640x480@15fps JPEG = ~0.5 MB/s, 50x less bandwidth!):**
```bash
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py
```

This will:
- ✅ Reduce bandwidth by 50x
- ✅ Reduce network congestion
- ✅ Enable remote viewing over WiFi/cellular
- ✅ Allow multiple camera streams
- ⚠️ Slightly increase CPU usage for encoding
- ⚠️ Require compressed-compatible subscribers

## 🐛 Troubleshooting

### "Camera open failed"
- Check if camera is connected: `ls -l /dev/video*`
- Check permissions: `sudo usermod -aG video $USER`
- Try different device: `device:=/dev/video1`

### H.264 node fails to start
- Install GStreamer: `sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good`
- Check available encoders: `gst-inspect-1.0 | grep h264enc`
- Use software fallback: `hardware_encoder:=x264`

### Compressed images not displaying
- Install image_transport plugins: `sudo apt install ros-${ROS_DISTRO}-compressed-image-transport`
- Check topic type: `ros2 topic info /camera/image_raw/compressed`

### High CPU usage with H.264
- Hardware encoder not being used - check detection with node logs
- Install hardware encoder packages for your platform
- Reduce resolution or bitrate

## 📝 Example Integration

Update your existing launch files:
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Include compressed camera in your robot launch
camera_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('mecanumbot_camera_stream'),
            'launch',
            'camera_compressed.launch.py'
        ])
    ]),
    launch_arguments={
        'jpeg_quality': '85',
        'fps': '15.0',
    }.items()
)
```

## 🎓 Additional Resources

- [ROS 2 image_pipeline documentation](http://wiki.ros.org/image_pipeline)
- [GStreamer documentation](https://gstreamer.freedesktop.org/documentation/)
- [JPEG quality guidelines](https://photo.stackexchange.com/questions/30243/what-quality-to-choose-when-converting-to-jpg)

---

**Need help?** Check the mecanumbot documentation or open an issue.
