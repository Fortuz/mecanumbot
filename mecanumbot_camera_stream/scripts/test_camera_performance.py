#!/usr/bin/env python3
"""
Camera Stream Performance Tester
Tests bandwidth and performance for different camera configurations.
"""

import subprocess
import time


class CameraTester:
    def __init__(self):
        self.process: subprocess.Popen | None = None

    def start_node(self, launch_cmd: str) -> bool:
        """Start a camera node."""
        print("\n🚀 Starting node...")
        try:
            self.process = subprocess.Popen(
                launch_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            time.sleep(3)

            if self.process.poll() is not None:
                print("❌ Node failed to start")
                return False

            print("✅ Node started successfully")
            return True
        except Exception as e:
            print(f"❌ Error: {e}")
            return False

    def stop_node(self):
        """Stop the camera node."""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None

    def measure_bandwidth(self, topic: str, duration: int = 10) -> tuple[float, float]:
        """Measure bandwidth and frequency."""
        print(f"📊 Measuring {topic} for {duration}s...")

        try:
            bw_result = subprocess.run(
                f"timeout {duration} ros2 topic bw {topic}",
                shell=True,
                capture_output=True,
                text=True,
            )

            hz_result = subprocess.run(
                f"timeout {duration} ros2 topic hz {topic}",
                shell=True,
                capture_output=True,
                text=True,
            )

            bandwidth_kbps = 0.0
            for line in bw_result.stdout.split("\n"):
                if "average:" in line.lower():
                    parts = line.split()
                    if len(parts) >= 2:
                        val = float(parts[1])
                        if "MB/s" in line:
                            bandwidth_kbps = val * 1024
                        elif "KB/s" in line:
                            bandwidth_kbps = val

            frequency = 0.0
            for line in hz_result.stdout.split("\n"):
                if "average rate:" in line.lower():
                    parts = line.split()
                    if len(parts) >= 3:
                        frequency = float(parts[2])

            print(
                f"   Bandwidth: {bandwidth_kbps:.1f} KB/s ({bandwidth_kbps/1024:.2f} MB/s)"
            )
            print(f"   Frequency: {frequency:.1f} Hz")

            return bandwidth_kbps, frequency

        except Exception as e:
            print(f"❌ Error: {e}")
            return 0.0, 0.0

    def test_configuration(self, name: str, launch_cmd: str, topic: str):
        """Test a configuration."""
        print(f"\n{'='*60}")
        print(f"Testing: {name}")
        print(f"{'='*60}")

        if not self.start_node(launch_cmd):
            return

        print("⏳ Waiting for topic...")
        for i in range(20):
            result = subprocess.run(
                f"ros2 topic list | grep -q '{topic}'", shell=True, capture_output=True
            )
            if result.returncode == 0:
                break
            time.sleep(0.5)
        else:
            print("❌ Topic not found")
            self.stop_node()
            return

        print("✅ Topic is publishing")
        bandwidth, frequency = self.measure_bandwidth(topic, duration=10)

        if frequency > 0:
            data_per_frame = bandwidth / frequency
            print(f"   Data per frame: {data_per_frame:.1f} KB")

        self.stop_node()
        time.sleep(2)


def main():
    print("🎥 Camera Stream Performance Tester")
    print("=" * 60)

    tester = CameraTester()

    tests = [
        {
            "name": "JPEG Compressed (640x480@15fps, Q=80)",
            "launch": "ros2 launch mecanumbot_camera_stream camera_compressed.launch.py",
            "topic": "/camera/image_raw/compressed",
        },
    ]

    try:
        for test in tests:
            tester.test_configuration(test["name"], test["launch"], test["topic"])
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted")
    finally:
        tester.stop_node()

    print("\n✅ Testing complete!")


if __name__ == "__main__":
    main()
