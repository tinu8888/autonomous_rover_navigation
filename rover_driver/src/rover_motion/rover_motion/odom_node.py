'''
# Import necessary libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

# Constants
PI = 3.141592
TICKS_PER_REVOLUTION = 620  # For reference purposes
WHEEL_RADIUS = 0.033  # Wheel radius in meters
WHEEL_BASE = 0.17  # Center of left tire to center of right tire
TICKS_PER_METER = 3100  # Original was 2800

# Distance both wheels have traveled
distanceLeft = 0.0
distanceRight = 0.0

# Flag to see if initial pose has been received
initialPoseReceived = False

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_node')

        self.odomNew = Odometry()
        self.odomOld = Odometry()

        # Set the initial pose
        self.odomOld.pose.pose.position.x = 0.0
        self.odomOld.pose.pose.position.y = 0.0
        self.odomOld.pose.pose.orientation.z = 0.00000000001

        # Set up publishers and subscribers
        self.subForRightCounts = self.create_subscription(
            Int16,
            'right_ticks',
            self.calc_right,
            10)

        self.subForLeftCounts = self.create_subscription(
            Int16,
            'left_ticks',
            self.calc_left,
            10)

        self.subInitialPose = self.create_subscription(
            PoseStamped,
            'initial_2d',
            self.set_initial_2d,
            10)

        self.odom_data_pub = self.create_publisher(Odometry, 'odom_data_euler', 10)
        self.odom_data_pub_quat = self.create_publisher(Odometry, 'odom_data_quat', 10)

        self.timer = self.create_timer(1/30.0, self.timer_callback)

    def set_initial_2d(self, msg):
        global initialPoseReceived
        self.odomOld.pose.pose.position.x = msg.pose.position.x
        self.odomOld.pose.pose.position.y = msg.pose.position.y
        self.odomOld.pose.pose.orientation.z = msg.pose.orientation.z
        initialPoseReceived = True

    def calc_left(self, msg):
        global distanceLeft
        static_last_count = 0
        if msg.data != 0 and static_last_count != 0:
            left_ticks = msg.data - static_last_count
            if left_ticks > 10000:
                left_ticks = 0 - (65535 - left_ticks)
            elif left_ticks < -10000:
                left_ticks = 65535 - left_ticks
            distanceLeft = left_ticks / TICKS_PER_METER
        static_last_count = msg.data

    def calc_right(self, msg):
        global distanceRight
        static_last_count = 0
        if msg.data != 0 and static_last_count != 0:
            right_ticks = msg.data - static_last_count
            if right_ticks > 10000:
                right_ticks = 0 - (65535 - right_ticks)
            elif right_ticks < -10000:
                right_ticks = 65535 - right_ticks
            distanceRight = right_ticks / TICKS_PER_METER
        static_last_count = msg.data

    def update_odom(self):
        global distanceLeft, distanceRight

        # Calculate the average distance
        cycle_distance = (distanceRight + distanceLeft) / 2

        # Calculate the number of radians the robot has turned since the last cycle
        cycle_angle = math.asin((distanceRight - distanceLeft) / WHEEL_BASE)

        # Average angle during the last cycle
        avg_angle = cycle_angle / 2 + self.odomOld.pose.pose.orientation.z

        if avg_angle > PI:
            avg_angle -= 2 * PI
        elif avg_angle < -PI:
            avg_angle += 2 * PI

        # Calculate the new pose (x, y, and theta)
        self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x + math.cos(avg_angle) * cycle_distance
        self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y + math.sin(avg_angle) * cycle_distance
        self.odomNew.pose.pose.orientation.z = cycle_angle + self.odomOld.pose.pose.orientation.z

        # Prevent lockup from a single bad cycle
        if math.isnan(self.odomNew.pose.pose.position.x) or math.isnan(self.odomNew.pose.pose.position.y):
            self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x
            self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y
            self.odomNew.pose.pose.orientation.z = self.odomOld.pose.pose.orientation.z

        # Make sure theta stays in the correct range
        if self.odomNew.pose.pose.orientation.z > PI:
            self.odomNew.pose.pose.orientation.z -= 2 * PI
        elif self.odomNew.pose.pose.orientation.z < -PI:
            self.odomNew.pose.pose.orientation.z += 2 * PI

        # Compute the velocity
        current_time = self.get_clock().now().to_msg()
        self.odomNew.header.stamp = current_time
        self.odomNew.twist.twist.linear.x = cycle_distance / (current_time.sec - self.odomOld.header.stamp.sec)
        self.odomNew.twist.twist.angular.z = cycle_angle / (current_time.sec - self.odomOld.header.stamp.sec)

        # Save the pose data for the next cycle
        self.odomOld.pose.pose.position.x = self.odomNew.pose.pose.position.x
        self.odomOld.pose.pose.position.y = self.odomNew.pose.pose.position.y
        self.odomOld.pose.pose.orientation.z = self.odomNew.pose.pose.orientation.z
        self.odomOld.header.stamp = current_time

        # Publish the odometry message
        self.odom_data_pub.publish(self.odomNew)

    def publish_quat(self):
        #q = tf_transformations.quaternion_from_euler(0, 0, self.odomNew.pose.pose.orientation.z)
        q = tf2_ros.QoSProfile(0, 0, self.odomNew.pose.pose.orientation.z)

        quat_odom = Odometry()
        quat_odom.header.stamp = self.odomNew.header.stamp
        quat_odom.header.frame_id = 'odom'
        quat_odom.child_frame_id = 'base_link'
        quat_odom.pose.pose.position.x = self.odomNew.pose.pose.position.x
        quat_odom.pose.pose.position.y = self.odomNew.pose.pose.position.y
        quat_odom.pose.pose.position.z = self.odomNew.pose.pose.position.z
        quat_odom.pose.pose.orientation.x = q[0]
        quat_odom.pose.pose.orientation.y = q[1]
        quat_odom.pose.pose.orientation.z = q[2]
        quat_odom.pose.pose.orientation.w = q[3]
        quat_odom.twist.twist.linear.x = self.odomNew.twist.twist.linear.x
        quat_odom.twist.twist.linear.y = self.odomNew.twist.twist.linear.y
        quat_odom.twist.twist.linear.z = self.odomNew.twist.twist.linear.z
        quat_odom.twist.twist.angular.x = self.odomNew.twist.twist.angular.x
        quat_odom.twist.twist.angular.y = self.odomNew.twist.twist.angular.y
        quat_odom.twist.twist.angular.z = self.odomNew.twist.twist.angular.z

        for i in range(36):
            if i in [0, 7, 14]:
                quat_odom.pose.covariance[i] = .01
            elif i in [21, 28, 35]:
                quat_odom.pose.covariance[i] += 0.1
            else:
                quat_odom.pose.covariance[i] = 0

        self.odom_data_pub_quat.publish(quat_odom)

    def timer_callback(self):
        if initialPoseReceived:
            self.update_odom()
            self.publish_quat()


def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomPublisher()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
#Shivam latest working June 5th 2:35am 
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


