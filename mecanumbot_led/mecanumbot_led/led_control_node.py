import rclpy
from rclpy.node import Node
import serial

from mecanumbot_msgs.srv  import GetLedStatus,SetLedStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


START_BYTE = 0xAA
FEEDBACK_START = 0xAB
REQUEST_BYTE = 0xAC

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
        b = ser.read(1)[0]
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
                    return (FL_mode,FL_color,FR_mode,FR_color,BL_mode,BL_color,BR_mode,BR_color)
                return None
    return None

class LedServiceNode(Node):
    def __init__(self):
        super().__init__('mecanumbot_led_service_node')

        # Initialize serial connection
        self.serial_port = serial.Serial('/dev/arduino_nano', 115200, timeout=1) # Udev rule should be set first

        self.callback_group = ReentrantCallbackGroup()
        # Create services
        self.last_feedback = None
        self.srv_set = self.create_service(SetLedStatus, 'set_led_status', self.set_led_status_callback,callback_group=self.callback_group)
        self.srv_get = self.create_service(GetLedStatus, 'get_led_status', self.get_led_status_callback, callback_group=self.callback_group)
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
            self.get_logger().info("Sending packet to Arduino")
            self.serial_port.write(packet)
            response.success = True
            response.message = "No error"
        except Exception as e:
            response.success = False
            response.message = e
            self.get_logger().error(e)
        return response


    def get_led_status_callback(self,request, response):
        try:
            self.serial_port.write(bytes([REQUEST_BYTE]))
            # Read fixed length response
            data = read_feedback(self.serial_port)
            if data is None or len(data) != 8:
                e = f"Incomplete packet: {len(data)} bytes"
                self.get_logger().error(e)
                (response.fl_mode, response.fl_color,
                response.fr_mode, response.fr_color,
                response.br_mode, response.br_color,
                response.bl_mode, response.bl_color) = (404,404,404,404,404,404,404,404)
                return response


            fl_m  = data[0]
            fl_c  = data[1]
            fr_m  = data[2]
            fr_c  = data[3]
            bl_m  = data[4]
            bl_c  = data[5]
            br_m  = data[6]
            br_c  = data[7]

            # Assign to ROS response
            response.fl_mode  = fl_m
            response.fl_color = fl_c
            response.fr_mode  = fr_m
            response.fr_color = fr_c
            response.bl_mode  = bl_m
            response.bl_color = bl_c
            response.br_mode  = br_m
            response.br_color = br_c
            return response 

        except Exception as e:
            e = "Serial error: {e}"
            self.get_logger().error(e)
            response.success = False
            response.message = e
            return response


def main(args=None):
    rclpy.init(args=args)
    node = LedServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()