import serial
import pynmea2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(String, 'gps_odm', 100)
        self.serial_port = '/dev/ttyAMA3'  # Adjust as needed
        self.baud_rate = 9600
        self.timer_period = 1.0  # seconds
        self.get_logger().info("Starting GPS Publisher...")
        self.start_serial_read()

    def start_serial_read(self):
        try:
            with serial.Serial(self.serial_port, self.baud_rate, timeout=1) as ser:
                while rclpy.ok():  # Keep reading if ROS is active
                    line = ser.readline().decode('ascii', errors='replace').strip()
                    if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                        try:
                            msg = pynmea2.parse(line)
                            latitude = msg.latitude
                            longitude = msg.longitude
                            gps_data = f"Latitude: {latitude}, Longitude: {longitude}"
                            self.publish_gps_data(gps_data)
                        except pynmea2.ParseError as e:
                            self.get_logger().error(f"Parse error: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")

    def publish_gps_data(self, gps_data):
        msg = String()
        msg.data = gps_data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {gps_data}")

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
