import rclpy
from rclpy.node import Node
import serial

from mecanumbot_msgs.srv  import GetLedStatus, SetLedStatus

class LedServiceNode(Node):
    def __init__(self):
        super().__init__('mecanumbot_led_service_node')

        # Initialize serial connection
        self.serial_port = serial.Serial('/dev/arduino_nano', 115200, timeout=1) # Udev rule should be set first

        # Create services
        self.srv_set = self.create_service(SetLedStatus, 'set_led_status', self.set_led_status_callback)
        self.srv_get = self.create_service(GetLedStatus, 'get_led_status', self.get_led_status_callback)

    def set_led_status_callback(self, request, response):
        # Send command over serial
        data = f"{request.fl_mode},{request.fl_color},{request.fr_mode},{request.fr_color}," \
               f"{request.br_mode},{request.br_color},{request.bl_mode},{request.bl_color}" #\n
        
        self.serial_port.write(data.encode())

        # (Optional) wait for ACK or process response from microcontroller
        response.success = True
        response.message = "No error"
        self.get_logger().info(f"Set new data: {data}")
        return response

    def get_led_status_callback(self, request, response):
        try:
            # Send GET command over serial
            self.serial_port.write(b"GET\n")
            
            # Read the response line from the serial port
            raw = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Received raw data: {raw}")

            # Check length (expecting exactly 16 characters → 8 values)
            if len(raw) != 16:
                self.get_logger().error(f"Unexpected data length: {len(raw)}. Data: {raw}")
                return response  # Optionally set all fields to -1 or 0 to indicate error

            # Split into 2-character chunks
            chunks = [raw[i:i+2] for i in range(0, len(raw), 2)]

            # Convert to integers (base 16)
            values = [int(chunk, 16) for chunk in chunks]

            # Assign values to response fields
            (response.fl_mode, response.fl_color,
            response.fr_mode, response.fr_color,
            response.br_mode, response.br_color,
            response.bl_mode, response.bl_color) = values

        except Exception as e:
            self.get_logger().error(f"Error parsing serial data: {e}")
            # Optionally set all response fields to 0 or -1 here
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()