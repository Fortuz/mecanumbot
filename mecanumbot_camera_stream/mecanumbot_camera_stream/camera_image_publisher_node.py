#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraImagePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_image_publisher_node')

        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('topic_name', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('encoding', 'bgr8')

        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)

        self.declare_parameter('csi_sensor_id', 0)
        self.declare_parameter('csi_flip_method', 0)
        self.declare_parameter('csi_gstreamer_pipeline', '')

        self.declare_parameter('queue_size', 10)
        self.declare_parameter('reopen_delay_sec', 2.0)

        self.camera_backend = str(self.get_parameter('camera_backend').value).lower()
        self.device = str(self.get_parameter('device').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.encoding = str(self.get_parameter('encoding').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)

        self.csi_sensor_id = int(self.get_parameter('csi_sensor_id').value)
        self.csi_flip_method = int(self.get_parameter('csi_flip_method').value)
        self.csi_gstreamer_pipeline = str(self.get_parameter('csi_gstreamer_pipeline').value)

        self.queue_size = int(self.get_parameter('queue_size').value)
        self.reopen_delay_sec = float(self.get_parameter('reopen_delay_sec').value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.topic_name, self.queue_size)

        self.capture: Optional[cv2.VideoCapture] = None
        self.last_open_attempt = 0.0

        self._open_capture_if_needed(force=True)

        timer_period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'Streaming to {self.topic_name} with backend={self.camera_backend}, '
            f'device={self.device}, size={self.width}x{self.height}@{self.fps:.1f}'
        )

    def _build_csi_pipeline(self) -> str:
        if self.csi_gstreamer_pipeline.strip():
            return self.csi_gstreamer_pipeline

        return (
            'nvarguscamerasrc sensor-id=' + str(self.csi_sensor_id) + ' ! '
            'video/x-raw(memory:NVMM), width=' + str(self.width) + ', height=' + str(self.height) +
            ', framerate=' + str(int(self.fps)) + '/1 ! '
            'nvvidconv flip-method=' + str(self.csi_flip_method) + ' ! '
            'video/x-raw, width=' + str(self.width) + ', height=' + str(self.height) + ', format=BGRx ! '
            'videoconvert ! video/x-raw, format=BGR ! appsink drop=1'
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

    def _timer_callback(self) -> None:
        self._open_capture_if_needed()
        if self.capture is None:
            return

        success, frame = self.capture.read()
        if not success or frame is None:
            self.get_logger().warn('Failed to read frame. Reopening camera.')
            self._release_capture()
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def _release_capture(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def destroy_node(self) -> bool:
        self._release_capture()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
