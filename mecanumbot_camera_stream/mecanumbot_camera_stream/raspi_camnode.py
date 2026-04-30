import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from picamera2 import Picamera2

class CameraNode(Node):
    def __init__(self):
        super().__init__('picamera2_node')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration())
        self.picam2.start()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frame = self.picam2.capture_array()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()