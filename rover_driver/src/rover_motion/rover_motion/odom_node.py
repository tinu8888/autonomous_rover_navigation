import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/pwm_signals', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Define the GPIO pin numbers where the PWM signals are connected
        self.PWM_PIN_RB = 13
        self.PWM_PIN_LB = 12
        self.PWM_PIN_RF = 20
        self.PWM_PIN_LF = 16

        # Set the GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWM_PIN_RB, GPIO.IN)
        GPIO.setup(self.PWM_PIN_LB, GPIO.IN)
        GPIO.setup(self.PWM_PIN_RF, GPIO.IN)
        GPIO.setup(self.PWM_PIN_LF, GPIO.IN)

    def measure_pwm(self, pin, timeout=1.0):
        start_time = time.time()
        end_time = start_time
        high_time = low_time = 0

        # Measure the high time
        while GPIO.input(pin) == 0:
            start_time = time.time()
            if (start_time - end_time) > timeout:
                return 0.0, 0.0  # Timeout occurred, return default values
        while GPIO.input(pin) == 1:
            end_time = time.time()
            if (end_time - start_time) > timeout:
                return 0.0, 0.0  # Timeout occurred, return default values
        high_time = end_time - start_time

        # Measure the low time
        while GPIO.input(pin) == 1:
            start_time = time.time()
            if (start_time - end_time) > timeout:
                return 0.0, 0.0  # Timeout occurred, return default values
        while GPIO.input(pin) == 0:
            end_time = time.time()
            if (end_time - start_time) > timeout:
                return 0.0, 0.0  # Timeout occurred, return default values
        low_time = end_time - start_time

        # Calculate frequency and duty cycle
        period = high_time + low_time
        frequency = 1.0 / period if period > 0 else 0.0
        duty_cycle = (high_time / period) * 100.0 if period > 0 else 0.0

        return frequency, duty_cycle

    def timer_callback(self):
        frequency_RB, duty_cycle_RB = self.measure_pwm(self.PWM_PIN_RB)
        frequency_LB, duty_cycle_LB = self.measure_pwm(self.PWM_PIN_LB)
        frequency_RF, duty_cycle_RF = self.measure_pwm(self.PWM_PIN_RF)
        frequency_LF, duty_cycle_LF = self.measure_pwm(self.PWM_PIN_LF)

        msg = Float32MultiArray()
        msg.data = [
            float(frequency_RB), float(duty_cycle_RB),
            float(frequency_LB), float(duty_cycle_LB),
            float(frequency_RF), float(duty_cycle_RF),
            float(frequency_LF), float(duty_cycle_LF)
        ]
        #msg.data = float(frequency_RB)
        self.publisher_.publish(msg)

        self.get_logger().info("************************************************")
        self.get_logger().info(f"Left f:{frequency_LF:.2f}, lb: {frequency_LB:.2f}, rf: {frequency_RF:.2f}, rb: {frequency_RB:.2f}.")

        if abs((frequency_RB + frequency_RB) / 2) - 50 > abs((frequency_LB + frequency_LF) / 2):
            self.get_logger().info("Turning Left !!!!!!!!!!")
            if frequency_LF - 50 > frequency_LB:
                self.get_logger().info(f"Left Freq: {frequency_LB:.2f} Hz, Duty cycle: {duty_cycle_LB:.2f}")
            elif frequency_LB - 50 > frequency_LF:
                self.get_logger().info(f"Left Freq: {frequency_LF:.2f} Hz, Duty cycle: {duty_cycle_LF:.2f}")
            else:
                self.get_logger().info(f"Left Freq: {(frequency_LF + frequency_LB) / 2:.2f} Hz, Duty cycle: {(duty_cycle_LB + duty_cycle_LF) / 2:.2f}")
            self.get_logger().info(f"RIGHT Freq: {(frequency_RF + frequency_RB) / 2:.2f} Hz, Duty cycle: {(duty_cycle_RB + duty_cycle_RF) / 2:.2f}")
        elif abs((frequency_RB + frequency_RB) / 2) < abs((frequency_LB + frequency_LF) / 2) - 50:
            self.get_logger().info("Turning RIGHT !!!!!!!!!!")
            if frequency_RF - 50 > frequency_RB:
                self.get_logger().info(f"RIGHT Freq: {frequency_RB:.2f} Hz, Duty cycle: {duty_cycle_RB:.2f}")
            elif frequency_RB - 50 > frequency_RF:
                self.get_logger().info(f"RIGHT Freq: {frequency_RF:.2f} Hz, Duty cycle: {duty_cycle_RF:.2f}")
            else:
                self.get_logger().info(f"RIGHT Freq: {(frequency_RF + frequency_RB) / 2:.2f} Hz, Duty cycle: {(duty_cycle_RB + duty_cycle_RF) / 2:.2f}")
            self.get_logger().info(f"Left Freq: {(frequency_LF + frequency_LB) / 2:.2f} Hz, Duty cycle: {(duty_cycle_LB + duty_cycle_LF) / 2:.2f}")
        else:
            self.get_logger().info(f"Straight Freq: {(frequency_LF + frequency_LB + frequency_RF + frequency_RB) / 2:.2f} Hz, Duty cycle: {(duty_cycle_LB + duty_cycle_LF + duty_cycle_RB + duty_cycle_RF) / 2:.2f}")

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomPublisher()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


