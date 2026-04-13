#!/usr/bin/env python3

from __future__ import annotations

import subprocess
import threading
import time
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class H264CameraPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('h264_camera_publisher_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('topic_name', '/camera/image_raw/h264')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('bitrate', 2000000)

        self.declare_parameter('hardware_encoder', 'auto')
        self.declare_parameter('custom_pipeline', '')

        self.declare_parameter('queue_size', 5)

        self.device = str(self.get_parameter('device').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.bitrate = int(self.get_parameter('bitrate').value)
        self.hardware_encoder = str(self.get_parameter('hardware_encoder').value).lower()
        self.custom_pipeline = str(self.get_parameter('custom_pipeline').value)
        self.queue_size = int(self.get_parameter('queue_size').value)

        self.publisher = self.create_publisher(CompressedImage, self.topic_name, self.queue_size)

        self.gst_process: Optional[subprocess.Popen] = None
        self.reader_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

        self._start_gstreamer_pipeline()

        self.get_logger().info(
            f'H.264 hardware streaming to {self.topic_name}, '
            f'size={self.width}x{self.height}@{self.fps:.1f}, '
            f'bitrate={self.bitrate/1000:.0f}kbps, encoder={self.hardware_encoder}'
        )

    def _detect_hardware_encoder(self) -> str:
        try:
            result = subprocess.run(
                ['gst-inspect-1.0', 'nvv4l2h264enc'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0:
                return 'nvenc'
        except Exception:
            pass

        try:
            result = subprocess.run(
                ['gst-inspect-1.0', 'vaapih264enc'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0:
                return 'vaapi'
        except Exception:
            pass

        try:
            result = subprocess.run(
                ['gst-inspect-1.0', 'omxh264enc'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0:
                return 'omx'
        except Exception:
            pass

        self.get_logger().warn('No hardware encoder detected, falling back to software (x264)')
        return 'x264'

    def _build_gstreamer_pipeline(self) -> str:
        if self.custom_pipeline.strip():
            return self.custom_pipeline

        encoder = self.hardware_encoder
        if encoder == 'auto':
            encoder = self._detect_hardware_encoder()

        device_str = self.device if not self.device.isdigit() else f'/dev/video{self.device}'

        if encoder == 'nvenc':
            pipeline = (
                f'v4l2src device={device_str} ! '
                f'video/x-raw, width={self.width}, height={self.height}, framerate={int(self.fps)}/1 ! '
                f'nvvidconv ! '
                f'video/x-raw(memory:NVMM) ! '
                f'nvv4l2h264enc bitrate={self.bitrate} maxperf-enable=1 ! '
                f'video/x-h264, stream-format=byte-stream ! '
                f'fdsink fd=1'
            )
        elif encoder == 'vaapi':
            pipeline = (
                f'v4l2src device={device_str} ! '
                f'video/x-raw, width={self.width}, height={self.height}, framerate={int(self.fps)}/1 ! '
                f'vaapih264enc bitrate={self.bitrate//1000} rate-control=cbr ! '
                f'video/x-h264, stream-format=byte-stream ! '
                f'fdsink fd=1'
            )
        elif encoder == 'omx':
            pipeline = (
                f'v4l2src device={device_str} ! '
                f'video/x-raw, width={self.width}, height={self.height}, framerate={int(self.fps)}/1 ! '
                f'omxh264enc target-bitrate={self.bitrate} ! '
                f'video/x-h264, stream-format=byte-stream ! '
                f'fdsink fd=1'
            )
        else:
            pipeline = (
                f'v4l2src device={device_str} ! '
                f'video/x-raw, width={self.width}, height={self.height}, framerate={int(self.fps)}/1 ! '
                f'x264enc bitrate={self.bitrate//1000} speed-preset=ultrafast tune=zerolatency ! '
                f'video/x-h264, stream-format=byte-stream ! '
                f'fdsink fd=1'
            )

        return pipeline

    def _start_gstreamer_pipeline(self) -> None:
        pipeline = self._build_gstreamer_pipeline()
        self.get_logger().info(f'Starting GStreamer pipeline: {pipeline}')

        try:
            self.gst_process = subprocess.Popen(
                ['gst-launch-1.0', '-q'] + pipeline.split(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )

            self.reader_thread = threading.Thread(target=self._reader_worker, daemon=True)
            self.reader_thread.start()

        except Exception as e:
            self.get_logger().error(f'Failed to start GStreamer pipeline: {e}')

    def _reader_worker(self) -> None:
        if self.gst_process is None or self.gst_process.stdout is None:
            return

        h264_buffer = bytearray()
        frame_count = 0

        while not self.stop_event.is_set():
            try:
                chunk = self.gst_process.stdout.read(4096)
                if not chunk:
                    self.get_logger().warn('GStreamer pipeline ended')
                    break

                h264_buffer.extend(chunk)

                nal_start = self._find_nal_unit(h264_buffer)
                if nal_start >= 0:
                    frame_count += 1
                    frame_data = bytes(h264_buffer[:nal_start])
                    h264_buffer = h264_buffer[nal_start:]

                    if len(frame_data) > 0:
                        msg = CompressedImage()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self.frame_id
                        msg.format = 'h264'
                        msg.data = frame_data
                        self.publisher.publish(msg)

            except Exception as e:
                self.get_logger().error(f'Error reading H.264 stream: {e}')
                break

    def _find_nal_unit(self, buffer: bytearray) -> int:
        if len(buffer) < 4:
            return -1

        for i in range(1, len(buffer) - 3):
            if buffer[i] == 0 and buffer[i+1] == 0 and buffer[i+2] == 0 and buffer[i+3] == 1:
                return i
            if buffer[i] == 0 and buffer[i+1] == 0 and buffer[i+2] == 1:
                return i

        return -1

    def destroy_node(self) -> bool:
        self.stop_event.set()

        if self.gst_process is not None:
            self.gst_process.terminate()
            try:
                self.gst_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.gst_process.kill()

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=2.0)

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = H264CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
