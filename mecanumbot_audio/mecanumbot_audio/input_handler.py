#!/usr/bin/env python3

from __future__ import annotations

import array
import re
import subprocess
import sys
import threading
import time

import rclpy
from mecanumbot_msgs.msg import AudioData
from rclpy.node import Node


class AudioInputHandler(Node):
    def __init__(self) -> None:
        super().__init__("audio_input_handler")

        self.declare_parameter("device", "auto")
        self.declare_parameter("topic_name", "audio_input")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("sample_format", "S16_LE")
        self.declare_parameter("reopen_delay_sec", 1.0)
        self.declare_parameter("device_scan_timeout_sec", 2.0)

        self.device = str(self.get_parameter("device").value).strip()
        self.topic_name = str(self.get_parameter("topic_name").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.channels = int(self.get_parameter("channels").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.sample_format = str(self.get_parameter("sample_format").value).upper()
        self.reopen_delay_sec = float(self.get_parameter("reopen_delay_sec").value)
        self.device_scan_timeout_sec = float(
            self.get_parameter("device_scan_timeout_sec").value
        )

        self.publisher = self.create_publisher(AudioData, self.topic_name, 10)
        self.stop_event = threading.Event()
        self.worker_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(
            f"Streaming microphone audio to {self.topic_name} with "
            f"device={self.device}, sample_rate={self.sample_rate}, "
            f"channels={self.channels}, chunk_size={self.chunk_size}, "
            f"format={self.sample_format}"
        )

    def _list_capture_devices(self) -> list[tuple[int, int, str]]:
        try:
            result = subprocess.run(
                ["arecord", "-l"],
                capture_output=True,
                text=True,
                timeout=self.device_scan_timeout_sec,
                check=False,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(f"Unable to enumerate audio capture devices: {exc}")
            return []

        capture_devices: list[tuple[int, int, str]] = []
        for line in result.stdout.splitlines():
            line = line.strip()
            match = re.search(r"card (\d+):\s*(.+), device (\d+):\s*(.+)", line)
            if match:
                card_index = int(match.group(1))
                device_index = int(match.group(3))
                capture_devices.append((card_index, device_index, line))
        return capture_devices

    def _resolve_device(self) -> str:
        if self.device and self.device.lower() != "auto":
            return self.device

        capture_devices = self._list_capture_devices()
        if capture_devices:
            card_index, device_index, description = capture_devices[0]
            selected_device = f"plughw:{card_index},{device_index}"
            self.get_logger().info(
                f"Auto-selected audio capture device {selected_device} from {description}"
            )
            return selected_device

        self.get_logger().warn(
            "No capture device detected. Falling back to default ALSA input."
        )
        return "default"

    def _build_command(self, device: str) -> list[str]:
        return [
            "arecord",
            "-q",
            "-D",
            device,
            "-f",
            self.sample_format,
            "-c",
            str(self.channels),
            "-r",
            str(self.sample_rate),
            "-t",
            "raw",
        ]

    def _capture_loop(self) -> None:
        bytes_per_sample = 2
        chunk_bytes = self.chunk_size * self.channels * bytes_per_sample

        while not self.stop_event.is_set():
            device = self._resolve_device()
            command = self._build_command(device)
            self.get_logger().info(
                f'Starting audio capture with command: {" ".join(command)}'
            )

            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                )
            except FileNotFoundError:
                self.get_logger().error(
                    "arecord is not installed or not available on PATH."
                )
                return
            except Exception as exc:  # pragma: no cover - defensive runtime guard
                self.get_logger().error(f"Failed to start audio capture: {exc}")
                time.sleep(self.reopen_delay_sec)
                continue

            try:
                assert process.stdout is not None
                while not self.stop_event.is_set():
                    raw_bytes = process.stdout.read(chunk_bytes)
                    if not raw_bytes:
                        break

                    pcm_samples = array.array("h")
                    pcm_samples.frombytes(raw_bytes)
                    if sys.byteorder != "little":
                        pcm_samples.byteswap()

                    audio_samples = array.array(
                        "f", (sample / 32768.0 for sample in pcm_samples)
                    )

                    message = AudioData()
                    message.data = audio_samples
                    self.publisher.publish(message)
            finally:
                self._stop_process(process)

            if not self.stop_event.is_set():
                time.sleep(self.reopen_delay_sec)

    def _stop_process(self, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return

        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AudioInputHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
