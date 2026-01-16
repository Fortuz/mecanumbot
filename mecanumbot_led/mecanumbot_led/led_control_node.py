import rclpy
from rclpy.node import Node
import serial

from mecanumbot_msgs.srv  import GetLedStatus,SetLedStatus


START_BYTE = 0xAA
FEEDBACK_START = 0xAB


def build_packet(FL_mode,FL_color,FR_mode,FR_color,BL_mode,BL_color,BR_mode,BR_color, duration_ms):
    tL = duration_ms & 0xFF
    tH = (duration_ms >> 8) & 0xFF
    checksum = FL_mode ^ FL_color ^ FR_mode ^ FR_color ^ BL_mode ^ BL_color ^ BR_mode ^ BR_color ^ tL ^ tH
    return bytes([START_BYTE, FL_mode,FL_color,FR_mode,FR_color,BL_mode,BL_color,BR_mode,BR_color, tL, tH, checksum])


def read_feedback(ser):
    static = read_feedback.__dict__
    if "state" not in static:
        static["state"] = 0
        static["buf"] = []

    while ser.in_waiting:
        b = ord(ser.read(1))
        if static["state"] == 0:
            if b == FEEDBACK_START:
                static["buf"] = []
                static["state"] = 1
        elif static["state"] == 1:
            static["buf"].append(b)
            if len(static["buf"]) == 9:
                FL_mode,FL_color,FR_mode,FR_color,BL_mode,BL_color,BR_mode,BR_color,cs = static["buf"]
                static["state"] = 0
                if (FL_mode ^ FL_color ^ FR_mode ^ FR_color ^ BL_mode ^ BL_color ^ BR_mode ^ BR_color) == cs:
                    return (FL_mode ^ FL_color ^ FR_mode ^ FR_color ^ BL_mode ^ BL_color ^ BR_mode ^ BR_color)
                return None
    return None

class LedServiceNode(Node):
    def __init__(self):
        super().__init__('mecanumbot_led_service_node')

        # Initialize serial connection
        self.serial_port = serial.Serial('/dev/arduino_nano', 115200, timeout=1) # Udev rule should be set first

        # Create services
        self.srv_set = self.create_service(SetLedStatus, 'set_led_status', self.set_led_status_callback)
        self.srv_get = self.create_service(GetLedStatus, 'get_led_status', self.get_led_status_callback)
        self.duration_ms = 1000

    def set_led_status_callback(self, request, response):
        
        packet = build_packet(
            request.fl_mode,
            request.fl_color,
            request.fr_mode,
            request.fr_color,
            request.bl_mode,
            request.bl_color,
            request.br_mode,
            request.br_color,
            self.duration_ms
        )
        try:
            self.serial_port.write(packet)
            return response
        except Exception as e:
            return response


    def get_led_status_callback(self, request, response):
        try:
            self.serial_port.write(bytes([FEEDBACK_START]))  # example GET command byte

            # Read fixed length response
            data = self.serial_port.read(10)

            if len(data) != 10:
                self.get_logger().error(f"Incomplete packet: {len(data)} bytes")
                return response

            if data[0] != FEEDBACK_START:
                self.get_logger().error("Invalid START byte")
                return response

            fl_m  = data[1]
            fl_c  = data[2]
            fr_m  = data[3]
            fr_c  = data[4]
            bl_m  = data[5]
            bl_c  = data[6]
            br_m  = data[7]
            br_c  = data[8]
            cs    = data[9]

            # Verify checksum
            if (fl_m ^ fl_c ^ fr_m ^ fr_c ^ bl_m ^ bl_c ^ br_m ^ br_c) != cs:
                self.get_logger().error("Checksum mismatch")
                return response

            # Assign to ROS response
            response.fl_mode  = fl_m
            response.fl_color = fl_c
            response.fr_mode  = fr_m
            response.fr_color = fr_c
            response.bl_mode  = bl_m
            response.bl_color = bl_c
            response.br_mode  = br_m
            response.br_color = br_c

        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()