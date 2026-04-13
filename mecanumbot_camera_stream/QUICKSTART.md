# Quick Start Guide - Optimized Camera Streaming

## 🎯 **RECOMMENDED: Use JPEG Compressed**

This gives you **50x less bandwidth** with great quality!

```bash
# Start compressed camera (640x480@15fps, JPEG Q=80)
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py

# View the stream
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw/compressed
```

**Result:** ~0.5 MB/s instead of ~26 MB/s!

---

## 📋 All Available Options

### 1. **JPEG Compressed** ⭐ Best for most use cases
```bash
# Standard quality (640x480@15fps)
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py

# High quality (1280x720@15fps)
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_compressed_hq.yaml

# Custom quality
ros2 launch mecanumbot_camera_stream camera_compressed.launch.py jpeg_quality:=90
```

### 2. **H.264 Hardware** ⚡ Ultra-low bandwidth
```bash
# Standard (640x480@15fps, 2Mbps)
ros2 launch mecanumbot_camera_stream camera_h264.launch.py

# High quality (1280x720@30fps, 4Mbps)
ros2 launch mecanumbot_camera_stream camera_h264.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_h264_hq.yaml
```

### 3. **Raw Images** 🖼️ For local CV processing
```bash
# Low bandwidth (320x240@15fps)
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_low_bandwidth.yaml

# Medium (640x480@15fps)
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_medium.yaml

# High quality (1280x720@30fps) - original config
ros2 launch mecanumbot_camera_stream camera_image_publisher.launch.py \
    params_file:=$(ros2 pkg prefix mecanumbot_camera_stream)/share/mecanumbot_camera_stream/config/camera_high.yaml
```

---

## 🔍 Check Performance

```bash
# Check bandwidth
ros2 topic bw /camera/image_raw/compressed

# Check framerate
ros2 topic hz /camera/image_raw/compressed

# Run automated tests (if camera is connected)
python3 /path/to/scripts/test_camera_performance.py
```

---

## 🎨 Viewing Options

### JPEG Compressed
```bash
# Option 1: image_tools
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw/compressed

# Option 2: rqt_image_view
ros2 run rqt_image_view rqt_image_view /camera/image_raw/compressed

# Option 3: Decompress to raw
ros2 run image_transport republish compressed raw \
    --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/decompressed
```

### Raw Images
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---

## 📊 Bandwidth Comparison

| Config | Resolution | FPS | Bandwidth | Reduction |
|--------|-----------|-----|-----------|-----------|
| **Old (Raw High)** | 1280x720 | 30 | 26 MB/s | Baseline |
| **Raw Medium** | 640x480 | 15 | 13.5 MB/s | 2x |
| **Raw Low** | 320x240 | 15 | 3.5 MB/s | 7x |
| **JPEG Std** ⭐ | 640x480 | 15 | 0.5 MB/s | **50x** |
| **JPEG HQ** | 1280x720 | 15 | 1-2 MB/s | 13-26x |
| **H.264** | 640x480 | 15 | 0.25 MB/s | **100x** |
| **H.264 HQ** | 1280x720 | 30 | 0.5 MB/s | **50x** |

---

## 🔧 Troubleshooting

**Camera not found?**
```bash
ls -l /dev/video*
# Try different device: device:=/dev/video1
```

**H.264 not working?**
```bash
# Check GStreamer
gst-inspect-1.0 | grep h264enc

# Install if missing
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good
```

**Compressed images not showing?**
```bash
# Install image_transport
sudo apt install ros-${ROS_DISTRO}-compressed-image-transport
```

---

## 📖 Full Documentation

See `OPTIMIZATION_GUIDE.md` for complete details on:
- Configuration parameters
- Hardware encoder setup
- Integration examples
- Performance tuning

---

**TIP:** For remote viewing over WiFi, start with JPEG Compressed. It's the best balance of quality, compatibility, and bandwidth!
