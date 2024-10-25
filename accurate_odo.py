'''
How encoders work with calculating odometry on differential steer robot.
This example works with traveling over small distances. The father you go the more possible paths your able to have.

First assume our robot dimensions are:
(1/2) foot wheel diameter
3 foot wheel base
88 count encoder
Say the robot traveled 1000 counts on the right encoder and 700 on the left encoder.
Total travel time is 5 seconds

Each turn of the wheel can be calculated into a distance traveled by 2∗pi∗radius of the wheel (2*pi*(1/2)) = 3.14 feet

The right encoder made (1000/88) = 11.36
That is 11 full revolutions for 11*3.14 = 34.54 feet and .36 of a full revolution so (.36*2*pi) = 2.26 feet. Combine those 2 for a total distance traveled
Dr = 34.54+2.26 = 36.8 feet.
Velocity of the wheel
Vr=distance/time = 36.8/5=7.36 feet per second

The left encoder made (700/88) = 7.95
That is 7 full revolutions for 7*3.14=21.98 feet and .95 of a full revolution so .95*2*pi=5.97 feet. Combine those 2 for a total distance traveled
DL = 21.98+5.97=27.95 feet.
Velocity of the wheel
Vl=distance/time = 27.95/5=5.59 feet per second

The total distance travel then is just the average of the 2 distances
Dt=(DL+Dr)/2
Dt = 27.95+36.8)/2=32.375 feet
Average velocity of the robot
V=distance/time = 32.375/5=6.475 feet per second

As you can tell the right wheel has traveled farther than the left wheel so the robot has turned to the left.

The average angle the robot has traveled
theta=(RightEncoderDistance−LeftEncoderGistance)/wheelbase
theta = (36.8-27.97)/3 = 2.95 radians

Then the distance traveled in the xdirection=Dt∗sin(theta) and the distance traveled in the ydirection=Dt∗cos(theta)

X=32.375*sin(2.95)=6.16 feet
Y=32.375*cos(2.95)=31.78 feet
'''
'''
#Shivam latest working June 5th 2:35am 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time

WIDTH_WHEEL= 4 #centimeter
BASE_DISTANCE_BETWEEN_WHEELS = 26 #cm
ENCODER_COUNTER_ROD = 620 #assuming 620 encoder rodes as it gave 620, 0 to 1 rises in 1 rotation
#counter = 0 

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

        self.counter_LF = 0
        self.counter_RF = 0
        self.counter_LB = 0
        self.counter_RB = 0


        # Set the GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWM_PIN_RB, GPIO.IN)
        GPIO.setup(self.PWM_PIN_LB, GPIO.IN)
        GPIO.setup(self.PWM_PIN_RF, GPIO.IN)
        GPIO.setup(self.PWM_PIN_LF, GPIO.IN)

    def measure_pwm(self, pin, timeout=1.0):
        #global counter
        counter = 0
        start_time = time.time()
        last_state = GPIO.input(pin)

        while (time.time() - start_time) < timeout:
            current_state = GPIO.input(pin)
            if last_state == 0 and current_state == 1:
                counter += 1
            last_state = current_state

        return counter        

    def timer_callback(self):
        self.counter_RB = self.measure_pwm(self.PWM_PIN_RB)
        self.counter_LB = self.measure_pwm(self.PWM_PIN_LB)
        self.counter_RF = self.measure_pwm(self.PWM_PIN_RF)
        self.counter_LF = self.measure_pwm(self.PWM_PIN_LF)

        msg = Float32MultiArray()
        msg.data = [
            float(self.counter_RB ), float(self.counter_LB ),
            float(self.counter_RF ), float(self.counter_LF )
        ]
        #msg.data = float(frequency_RB)
        self.publisher_.publish(msg)

        self.get_logger().info("************************************************")
        
def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomPublisher()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

#Shivam latest working June 9th 12:00 am 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time

WIDTH_WHEEL= 4 #centimeter
DIAMETER_WHEEL = 15 #cm
BASE_DISTANCE_BETWEEN_WHEELS = 26 #cm
ENCODER_COUNTER_ROD = 620 #assuming 620 encoder rodes as it gave 620, 0 to 1 rises in 1 rotation
# It takes 2.7 sec to complete 1 rotation of the wheel at 10.0 speed 

class Accurate_odo(Node):
    def __init__(self):
        super().__init__('accurate_odo')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/pwm_signals', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Define the GPIO pin numbers where the PWM signals are connected
        self.PWM_PIN_RB = 13
        self.PWM_PIN_LB = 12
        self.PWM_PIN_RT = 20
        self.PWM_PIN_LT = 16

        self.counter_LT = 0
        self.counter_RT = 0
        self.counter_LB = 0
        self.counter_RB = 0


        # Set the GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWM_PIN_RB, GPIO.IN)
        GPIO.setup(self.PWM_PIN_LB, GPIO.IN)
        GPIO.setup(self.PWM_PIN_RT, GPIO.IN)
        GPIO.setup(self.PWM_PIN_LT, GPIO.IN)

    def measure_pwm(self, pin, timeout=0.5):
        #global counter
        counter = 0
        start_time = time.time()
        last_state = GPIO.input(pin)

        while (time.time() - start_time) < timeout:
            current_state = GPIO.input(pin)
            if last_state == 0 and current_state == 1:
                counter += 1
            last_state = current_state

        return counter        

    def timer_callback(self):
        self.counter_RB = self.measure_pwm(self.PWM_PIN_RB)
        self.counter_LB = self.measure_pwm(self.PWM_PIN_LB)
        self.counter_RT = self.measure_pwm(self.PWM_PIN_RT)
        self.counter_LT = self.measure_pwm(self.PWM_PIN_LT)

        msg = Float32MultiArray()
        msg.data = [
            float(self.counter_RB ), float(self.counter_LB ),
            float(self.counter_RT ), float(self.counter_LT )
        ]
        #msg.data = float(frequency_RT)
        self.publisher_.publish(msg)

        self.get_logger().info("************************************************")
        
def main(args=None):
    rclpy.init(args=args)
    odom_node = Accurate_odo()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

