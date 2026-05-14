#!/usr/bin/env python3

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def get_device_model():
    try:
        with open("/proc/device-tree/model", "r") as f:
            return f.read().strip().lower()
    except FileNotFoundError:
        return ""

MODEL = get_device_model()

if "raspberry pi" in MODEL:
    print("Running on Raspberry Pi")
elif "nvidia jetson" in MODEL:
    print("Running on Jetson")
else:
    print("Unknown device:", MODEL)


class CompressedCameraPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('compressed_camera_publisher_node')

        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('topic_name', '/camera/image_raw/compressed')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)

        self.declare_parameter('format', 'jpeg')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('png_level', 3)

        self.declare_parameter('csi_sensor_id', 0)
        self.declare_parameter('csi_flip_method', 2)
        self.declare_parameter('csi_gstreamer_pipeline', '')

        self.declare_parameter('queue_size', 5)
        self.declare_parameter('reopen_delay_sec', 2.0)
        self.declare_parameter('use_capture_thread', True)
        self.declare_parameter('publish_fps', 15.0)

        self.camera_backend = str(self.get_parameter('camera_backend').value).lower()
        self.device = str(self.get_parameter('device').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)

        self.format = str(self.get_parameter('format').value).lower()
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.png_level = int(self.get_parameter('png_level').value)

        self.csi_sensor_id = int(self.get_parameter('csi_sensor_id').value)
        self.csi_flip_method = int(self.get_parameter('csi_flip_method').value)
        self.csi_gstreamer_pipeline = str(self.get_parameter('csi_gstreamer_pipeline').value)

        self.queue_size = int(self.get_parameter('queue_size').value)
        self.reopen_delay_sec = float(self.get_parameter('reopen_delay_sec').value)
        self.use_capture_thread = bool(self.get_parameter('use_capture_thread').value)
        self.publish_fps = float(self.get_parameter('publish_fps').value)

        self.publisher = self.create_publisher(CompressedImage, self.topic_name, self.queue_size)

        self.capture: Optional[cv2.VideoCapture] = None
        self.last_open_attempt = 0.0
        self.capture_lock = threading.Lock()
        self.latest_frame = None
        self.latest_seq = 0
        self.last_published_seq = -1
        self.stop_event = threading.Event()
        self.capture_thread: Optional[threading.Thread] = None

        if self.format == 'jpeg':
            self.encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            self.format_string = 'jpeg'
        elif self.format == 'png':
            self.encode_params = [cv2.IMWRITE_PNG_COMPRESSION, self.png_level]
            self.format_string = 'png'
        else:
            self.get_logger().error(f'Unsupported format: {self.format}. Using jpeg.')
            self.encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            self.format_string = 'jpeg'

        if self.use_capture_thread:
            self.capture_thread = threading.Thread(target=self._capture_worker, daemon=True)
            self.capture_thread.start()
        else:
            self._open_capture_if_needed(force=True)

        publish_rate = self.publish_fps if self.publish_fps > 0.0 else self.fps
        timer_period = 1.0 / max(publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'Compressed streaming to {self.topic_name} with backend={self.camera_backend}, '
            f'device={self.device}, size={self.width}x{self.height}@{self.fps:.1f}, '
            f'publish_fps={publish_rate:.1f}, format={self.format_string}, '
            f'quality={self.jpeg_quality if self.format_string == "jpeg" else self.png_level}'
        )

    def _build_csi_pipeline(self) -> str:
        if self.csi_gstreamer_pipeline.strip():
            return self.csi_gstreamer_pipeline


        # Optimized for Raspberry Pi 5 (libcamera)
        if "raspberry pi" in MODEL:
             return ( f"libcamerasrc ! video/x-raw, width={self.width}, height={self.height}, format=RGBx ! videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=true sync=false"
            )
        if "nvidia jetson" in MODEL:
        # Optimized for JetPack 6 / Orin
            return (
                f'nvarguscamerasrc sensor-id={self.csi_sensor_id} ! '
                f'video/x-raw(memory:NVMM), width={self.width}, height={self.height}, '
                f'framerate={int(self.fps)}/1 ! '
                f'nvvidconv flip-method={self.csi_flip_method} ! '
                f'video/x-raw, format=BGRx ! '
                f'videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=true sync=false'
            )

    def _open_capture(self) -> Tuple[Optional[cv2.VideoCapture], str]:
        if self.camera_backend == 'usb':
            cap = self._open_usb_capture(self.device)
            return cap, f'usb:{self.device}'

        if self.camera_backend == 'csi':
            pipeline = self._build_csi_pipeline()
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                return cap, f'csi:{pipeline}'
            return None, 'csi(open_failed)'

        cap_usb = self._open_usb_capture(self.device)
        if cap_usb is not None and cap_usb.isOpened():
            return cap_usb, f'auto->usb:{self.device}'

        pipeline = self._build_csi_pipeline()
        cap_csi = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap_csi.isOpened():
            return cap_csi, f'auto->csi:{pipeline}'

        return None, 'auto(open_failed)'

    def _open_usb_capture(self, device: str) -> Optional[cv2.VideoCapture]:
        source = int(device) if device.isdigit() else device
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _open_capture_if_needed(self, force: bool = False) -> None:
        if self.capture is not None and self.capture.isOpened():
            return

        now = time.monotonic()
        if not force and (now - self.last_open_attempt) < self.reopen_delay_sec:
            return

        self.last_open_attempt = now
        self.capture, selected_source = self._open_capture()
        if self.capture is None or not self.capture.isOpened():
            self.capture = None
            self.get_logger().warn(
                'Camera open failed. Retrying automatically. '
                f'backend={self.camera_backend}, device={self.device}'
            )
            return

        self.get_logger().info(f'Camera opened successfully: {selected_source}')

    def _capture_worker(self) -> None:
        while not self.stop_event.is_set():
            self._open_capture_if_needed()
            if self.capture is None:
                time.sleep(0.05)
                continue

            success, frame = self.capture.read()
            if not success or frame is None:
                self.get_logger().warn('Failed to read frame in capture thread. Reopening camera.')
                self._release_capture()
                continue

            with self.capture_lock:
                self.latest_frame = frame
                self.latest_seq += 1

    def _timer_callback(self) -> None:
        if not self.use_capture_thread:
            self._open_capture_if_needed()
            if self.capture is None:
                return

            success, frame = self.capture.read()
            if not success or frame is None:
                self.get_logger().warn('Failed to read frame. Reopening camera.')
                self._release_capture()
                return
        else:
            with self.capture_lock:
                if self.latest_frame is None or self.latest_seq == self.last_published_seq:
                    return
                frame = self.latest_frame.copy()
                self.last_published_seq = self.latest_seq

        success, encoded = cv2.imencode(f'.{self.format_string}', frame, self.encode_params)
        if not success:
            self.get_logger().warn('Failed to encode frame')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = self.format_string
        msg.data = encoded.tobytes()
        self.publisher.publish(msg)

    def _release_capture(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=2.0)
        self._release_capture()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompressedCameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
