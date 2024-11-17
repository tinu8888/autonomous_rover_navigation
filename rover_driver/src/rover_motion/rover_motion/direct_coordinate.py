'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import serial
import time
from math import sqrt,atan2, sin, cos, degrees

#sabertooth_serial = serial.Serial('/dev/ttyTHS0', 9600, timeout=1)
sabertooth_serial = serial.Serial('/dev/pts/5', 9600, timeout=1)

x_pos = 0.0
y_pos = 0.0

NEW = False
NOW_ROTATE = False
MOVE = True
NEXT = False


class Check(Node):
    pose_x = 0.0
    pose_y = 0.0
    theta = 0.0

    pose_msg = Pose()

    def __init__(self):
        super().__init__('direct_coordinate')
        self.main_left = 0
        self.main_right = 0
        self.left_side = 0.0
        self.right_side = 0.0
        self.i = 0 
        self.t =0
        self.publish_goal = 0
        self.start = 0
        self.x = 1
        self.reach = 0

        self.threshold = 0.5
        self.theta_error = 0.1
        self.absolute = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.new_y = 0.0
        self.theta = 0.0
        self.theta_old = 0.0
        self.theta_new = 0.0
        self.theta_degree =0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.absolute = 0.0

        self.checksum = 0
        self.packet = 0

        self.address = 0

        self.kp = 0.0

        #self.sub = self.create_subscription(Twist, '/command', self.cmd_callback, 10)
        self.sub1 = self.create_subscription(Pose, '/pose', self.goal_pos, 10)
        self.pub = self.create_publisher(Pose, '/self_pose', 10)
        self.pub1 = self.create_publisher(Pose, "/goal_reached", 10)
        
    def calculate_checksum(self, address, command, value):
        return (address + command + int(value)) & 0b01111111
    
    def calculate_angle(self):
        self.theta_new = atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)
        if (abs(self.theta_new - self.theta_old) > self.theta_error):
            self.theta_degree = degrees(self.theta_new) - degrees(self.theta_old)
            self.theta = self.theta_new - self.theta_old
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
            self.theta_old = self.theta_new
            
        else :
            self.theta_new = self.theta_old
            self.theta_degree = 0.0
            self.theta = 0.0
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
        

    def rotate_bot(self):
        if (self.theta_degree != 0):
            print("Rotate_bot")
            rotation = Twist()
            rotation.linear.x = 0.0
            rotation.angular.z = 9.5
            self.cmd_callback(rotation)
            time.sleep(abs((3*self.theta_degree)/90))
            print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
        else:
            print("No rotation")
        
    def send_sabertooth_command(self, serial_port, address, command, value):
        self.checksum = self.calculate_checksum(address, command, value)
        self.packet = bytes([address, command, int(value), self.checksum])
        serial_port.write(self.packet)
        #print(f"Sent command to Sabertooth: Address: {address}, Command: {command}, Value: {int(value)}, Checksum: {self.checksum}")
    
    def stop_motors(self):
        """Send stop command to all motors."""
        for address in [128, 129]:
            for command in [0, 1, 4, 5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)
        print("Motors stopped.")
        
    def goal_pos(self, data):
        self.goal_x = data.position.x
        self.goal_y = data.position.y
        print(f"goal_x: {self.goal_x}, goal_y: {self.goal_y}")
        self.start = 0
        self.reach = 0
        self.absolute_()
        self.calculate_angle()
        self.rotate_bot()
        self.command_send()

    #def kp_(self):
    #    if (self.start == 0):
    #        if ((self.goal_x - self.pose_x)> 60):
    #            self.kp = 7
    #        elif ((self.goal_x - self.pose_x)> 40 and (self.goal_x - self.pose_x)  <= 60):
    #            self.kp = 6
    #        elif ((self.goal_x - self.pose_x)> 20 and (self.goal_x - self.pose_x)<= 40):
    #            self.kp = 5
    #        elif ((self.goal_x - self.pose_x)> 10 and (self.goal_x - self.pose_x) <= 20):
    #            self.kp = 3
    #        elif ((self.goal_x - self.pose_x) > 3 and (self.goal_x - self.pose_x)<= 10):
    #            self.kp = 2
    #        else:
    #
    #             self.kp = 1
    def kp_(self):
        kp_max = 7
        kp_min = 1
        distance = abs(self.goal_x - self.pose_x)

        # Assuming the maximum distance is 60 for scaling purposes
        max_distance = 60

        # Proportional control: The closer the robot is to the goal, the smaller the kp
        self.kp = kp_min + (kp_max - kp_min) * (distance / max_distance)
        self.kp = max(min(self.kp, kp_max), kp_min)  # Ensure kp is within bounds
        print(f"Calculated kp: {self.kp} for distance: {distance}")

    def absolute_(self):
        if self.start == 0:
            self.absolute = sqrt((self.goal_x - self.pose_x)**2 + (self.goal_y - self.pose_y)**2)
            print(f"absolute value: {self.absolute}")

    def command_send(self):

        while self.reach == 0:
            #self.rotate_bot()
            self.absolute_()
            if (self.start == 0):
                self.kp_()
                print(f"ROBOT pose (x,y): ({self.pose_x},{self.pose_y}) and kp: {self.kp}")
                if (self.absolute > self.threshold):
                    goal_data = Twist()
                    goal_data.linear.x = self.kp * 10.0
                    goal_data.angular.z = 0.0
                    print(f"speed: {goal_data.linear.x}")
                    self.pose_x = self.pose_x + (goal_data.linear.x)*cos(self.theta_new)/10
                    self.pose_y = self.pose_y + (goal_data.linear.x)*sin(self.theta_new)/10
                    self.cmd_callback(goal_data)
                else:
                    goal_data1 = Twist()
                    goal_data1.linear.x = 0.0
                    goal_data1.angular.z = 0.0
                    self.cmd_callback(goal_data1)

        self.stop_motors()
        

    def pose_callback(self, data):
        self.pub.publish(data)

    def cmd_callback(self, msg):
        if(self.start == 0):
            print("cmd received!!!")
            if (msg.linear.x != 0.0 and msg.angular.z == 0.0):
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                for address in [128, 129]:
                    if (address == 129):
                        if (self.left_side >= 0):
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            for command in [1, 5]:
                                true_left = abs(self.left_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if (self.right_side >= 0):
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            for command in [1, 5]:
                                true_right = abs(self.right_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                time.sleep(1)
                print(f"1robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)

            elif (msg.linear.x == 0.0 and msg.angular.z != 0.0):
                if (msg.angular.z > 0.0):
                    self.left_side = -int(abs(msg.angular.z))
                    self.right_side = int(abs(msg.angular.z))
                    self.main_left = int(self.main_left + self.left_side)
                    self.main_right = int(self.main_right + self.right_side)
                    self.main_left = max(min(self.main_left, 127), -127)
                    self.main_right = max(min(self.main_right, 127), -127)
                    for address in [128, 129]:
                        if (address == 129):
                            if (self.left_side >= 0):
                                for command in [0, 4]:
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                                    print(f"Rotation speed of left wheel: {self.left_side}")
                            else:
                                for command in [1, 5]:
                                    true_left = abs(self.left_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                        else:
                            if (self.right_side >= 0):
                                for command in [0, 4]:
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                            else:
                                for command in [1, 5]:
                                    true_right = abs(self.right_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                    print(f"2robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                    time.sleep(1)
                    self.stop_motors()
                    pose_msg = Pose()
                    pose_msg.position.x = float(self.pose_x)
                    pose_msg.position.y = float(self.pose_y)
                    self.pose_callback(pose_msg)
                        
                else:
                    #print(f"31robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                    self.left_side = int(abs(msg.angular.z))
                    self.right_side = -int(abs(msg.angular.z))
                    self.main_left = int(self.main_left + self.left_side)
                    self.main_right = int(self.main_right + self.right_side)
                    self.main_left = max(min(self.main_left, 127), -127)
                    self.main_right = max(min(self.main_right, 127), -127)
                    for address in [128, 129]:
                        if (address == 129):
                            if (self.left_side >= 0):
                                for command in [0, 4]:
                                    #print("1")
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                            else:
                                for command in [1, 5]:
                                    #print("11")
                                    true_left = abs(self.left_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                        else:
                            if (self.right_side >= 0):
                                #print("2")
                                for command in [0, 4]:
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                            else:
                                for command in [1, 5]:
                                    #print("22")
                                    true_right = abs(self.right_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                                    #print(' out 22')
                    time.sleep(1)
                    print(f"3robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                    pose_msg = Pose()
                    pose_msg.position.x = float(self.pose_x)
                    pose_msg.position.y = float(self.pose_y)
                    self.pose_callback(pose_msg)
            print(self.absolute < self.threshold)
            if ((self.absolute < self.threshold)):
                print(f"Goal reached!!!!!!!!!!, x pos: {self.pose_x}, y pos: {self.pose_y}.")
                self.start = 1
                self.t = 0
                self.x = 1
                self.reach = 1
                reached = Pose()
                reached.position.x = 1000.0
                self.pub1.publish(reached)
                self.publish_goal = 1

def stop_fun(self):
    print("entering stop_fun")
    while True:
        for address in [128,129]:
            for command in [0,1,4,5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)

def main(args=None):
    rclpy.init(args=args)
    check = Check()
    try:
        rclpy.spin(check)
    except KeyboardInterrupt:
        check.stop_motors()
        print("Keyboard Interrupt detected. Stopping motors.")
    finally:
        check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

# Shivam latest working June 5th 2:34 am latest
'''
#Shivam Jusne3 8:00 pm

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import Float32MultiArray
import serial
import time
from math import sqrt,atan2, sin, cos, degrees

#sabertooth_serial = serial.Serial('/dev/ttyTHS0', 9600, timeout=1)
#sabertooth_serial = serial.Serial('/dev/pts/5', 9600, timeout=1)
sabertooth_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)

x_pos = 0.0
y_pos = 0.0

NEW = False
NOW_ROTATE = False
MOVE = True
NEXT = False


class Check(Node):
    pose_x = 0.0
    pose_y = 0.0
    theta = 0.0

    pose_msg = Pose()

    def __init__(self):
        super().__init__('direct_coordinate')
        self.main_left = 0
        self.main_right = 0
        self.left_side = 0.0
        self.right_side = 0.0
        self.i = 0 
        self.t =0
        self.publish_goal = 0
        self.start = 0
        self.x = 1
        self.reach = 0

        self.right_bot = 0.0
        self.left_bot = 0.0
        self.right_top = 0.0
        self.left_top = 0.0

        self.threshold = 0.5
        self.theta_error = 0.2
        self.absolute = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.new_y = 0.0
        self.theta = 0.0
        self.goal_angle =0.0
        self.theta_old = 0.0
        self.theta_new = 0.0
        self.theta_degree =0.0
        self.rotate_goal_true = 0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.absolute = 0.0

        self.checksum = 0
        self.packet = 0

        self.address = 0

        self.kp = 0.0

        self.sub = self.create_subscription(Float32MultiArray, '/pwm_signals', self.pwm_callback, 10)
        self.sub1 = self.create_subscription(Pose, '/pose', self.goal_pos, 10)
        self.pub = self.create_publisher(Pose, '/self_pose', 10)
        self.pub1 = self.create_publisher(Pose, "/goal_reached", 10)

        self.timer = self.create_timer(0.1, self.command_send)
        
    def pwm_callback(self,msg):
        print(msg)
        self.right_bot = msg.data[0]
        self.left_bot = msg.data[2]
        self.right_top = msg.data[4]
        self.left_bot = msg.data[6]
        print(f"({self.right_bot},{self.left_bot},{self.right_top},{self.left_bot})")

    def calculate_checksum(self, address, command, value):
        return (address + command + int(value)) & 0b01111111
    
    def calculate_angle(self):
        self.theta_new = atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)
        print(self.theta_new)
        if (abs(self.theta_new - self.theta_old) > self.theta_error):
            self.theta_degree = degrees(self.theta_new) - degrees(self.theta_old)
            self.theta = self.theta_new - self.theta_old
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
            self.theta_old = self.theta_new
            
        elif (abs(self.theta_new - self.theta_old) < self.theta_error):
            self.theta_degree = degrees(self.theta_new) - degrees(self.theta_old)
            self.theta = self.theta_new - self.theta_old
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
            self.theta_old = self.theta_new

        else :
            self.theta_new = self.theta_old
            self.theta_degree = 0.0
            self.theta = 0.0
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
        

    def rotate_bot(self):
        if (self.theta_degree != 0):
            print("Rotate_bot")
            rotation = Twist()
            if (self.theta_degree > 0):
                rotation.linear.x = 0.0
                rotation.angular.z = 10.0
                self.cmd_callback(rotation)
                time.sleep(abs((3*self.theta_degree)/90))
                print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
                self.right_bot = 0.0
                self.left_bot = 0.0
                self.right_top = 0.0
                self.left_top = 0.0
            else :
                rotation.linear.x = 0.0
                rotation.angular.z = -10.0
                self.cmd_callback(rotation)
                time.sleep(abs((3*self.theta_degree)/90))
                print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
                self.right_bot = 0.0
                self.left_bot = 0.0
                self.right_top = 0.0
                self.left_bot = 0.0
        else:
            print("No rotation")
        #if (self.rotate_goal_true == 1):
        #   print("Goal Rotate")
        #    rotation = Twist()
        #    rotation.linear.x = 0.0
        #    rotation.angular.z = 9.5
        #    self.cmd_callback(rotation)
        #    time.sleep(abs((3*self.goal_angle)/90))
        #    print(f"time.sleep: {abs((3*self.goal_angle)/90)}")
        #    self.rotate_goal_true = 0

        
    def send_sabertooth_command(self, serial_port, address, command, value):
        self.checksum = self.calculate_checksum(address, command, value)
        self.packet = bytes([address, command, int(value), self.checksum])
        serial_port.write(self.packet)
        #print(f"Sent command to Sabertooth: Address: {address}, Command: {command}, Value: {int(value)}, Checksum: {self.checksum}")
    
    def stop_motors(self):
        """Send stop command to all motors."""
        for address in [128, 129]:
            for command in [0, 1, 4, 5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)
        print("Motors stopped.")
        
    def goal_pos(self, data):
        self.goal_x = data.position.x
        self.goal_y = data.position.y
        #self.goal_angle = data.orientation.z
        print(f"goal_x: {self.goal_x}, goal_y: {self.goal_y}")
        self.start = 0
        self.reach = 0
        self.absolute_()
        self.calculate_angle()
        self.rotate_bot()
        self.stop_motors()
        self.command_send()

    def kp_(self):
        if (self.start == 0):
            if (abs(self.goal_x - self.pose_x)> 60):
                self.kp = 7
            elif (abs(self.goal_x - self.pose_x)> 40 and abs(self.goal_x - self.pose_x)  <= 60):
                self.kp = 6
            elif (abs(self.goal_x - self.pose_x)> 20 and abs(self.goal_x - self.pose_x)<= 40):
                self.kp = 5
            elif (abs(self.goal_x - self.pose_x)> 10 and abs(self.goal_x - self.pose_x) <= 20):
                self.kp = 3
            elif (abs(self.goal_x - self.pose_x) > 3 and abs(self.goal_x - self.pose_x)<= 10):
                self.kp = 2
            else:
    
                 self.kp = 1
    
    def absolute_(self):
        if self.start == 0:
            self.absolute = sqrt((self.goal_x - self.pose_x)**2 + (self.goal_y - self.pose_y)**2)
            print(f"absolute value: {self.absolute}")

    def command_send(self):

        #while self.reach == 0:
        if self.reach == 0:
            #self.rotate_bot()
            self.absolute_()
            if (self.start == 0):
                self.kp_()
                print(f"ROBOT pose (x,y): ({self.pose_x},{self.pose_y}) and kp: {self.kp}")
                if (self.absolute > self.threshold):
                    goal_data = Twist()
                    goal_data.linear.x = self.kp * 10.0
                    goal_data.angular.z = 0.0
                    print(f"speed: {goal_data.linear.x}")
                    if self.right_bot!=0 and self.right_top!=0 and self.left_bot!=0 and self.left_top!=0:
                        self.pose_x = self.pose_x + (goal_data.linear.x)*cos(self.theta_new)/10
                        self.pose_y = self.pose_y + (goal_data.linear.x)*sin(self.theta_new)/10
                    else:
                        self.pose_x = self.pose_x 
                        self.pose_y = self.pose_y
                    self.cmd_callback(goal_data)
                else:
                    goal_data1 = Twist()
                    goal_data1.linear.x = 0.0
                    goal_data1.angular.z = 0.0
                    self.cmd_callback(goal_data1)

        self.stop_motors()
        

    def pose_callback(self, data):
        self.pub.publish(data)

    def cmd_callback(self, msg):
        if(self.start == 0):
            print("cmd received!!!")
            if (msg.linear.x != 0.0 and msg.angular.z == 0.0):
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                for address in [128, 129]:
                    if (address == 129):
                        if (self.left_side >= 0):
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            for command in [1, 5]:
                                true_left = abs(self.left_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if (self.right_side >= 0):
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            for command in [1, 5]:
                                true_right = abs(self.right_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                time.sleep(1)
                print(f"1robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)

            elif (msg.linear.x == 0.0 and msg.angular.z != 0.0):
                if (msg.angular.z > 0.0):
                    self.left_side = -int(abs(msg.angular.z))
                    self.right_side = int(abs(msg.angular.z))
                    self.main_left = int(self.main_left + self.left_side)
                    self.main_right = int(self.main_right + self.right_side)
                    self.main_left = max(min(self.main_left, 127), -127)
                    self.main_right = max(min(self.main_right, 127), -127)
                    for address in [128, 129]:
                        if (address == 129):
                            if (self.left_side >= 0):
                                for command in [0, 4]:
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                                    print(f"Rotation speed of left wheel: {self.left_side}")
                            else:
                                for command in [1, 5]:
                                    true_left = abs(self.left_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                        else:
                            if (self.right_side >= 0):
                                for command in [0, 4]:
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                            else:
                                for command in [1, 5]:
                                    true_right = abs(self.right_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                    print(f"2robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                    time.sleep(1)
                    #self.stop_motors()
                    pose_msg = Pose()
                    pose_msg.position.x = float(self.pose_x)
                    pose_msg.position.y = float(self.pose_y)
                    self.pose_callback(pose_msg)
                        
                else:
                    #print(f"31robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                    self.left_side = int(abs(msg.angular.z))
                    self.right_side = -int(abs(msg.angular.z))
                    self.main_left = int(self.main_left + self.left_side)
                    self.main_right = int(self.main_right + self.right_side)
                    self.main_left = max(min(self.main_left, 127), -127)
                    self.main_right = max(min(self.main_right, 127), -127)
                    for address in [128, 129]:
                        if (address == 129):
                            if (self.left_side >= 0):
                                for command in [0, 4]:
                                    #print("1")
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                            else:
                                for command in [1, 5]:
                                    #print("11")
                                    true_left = abs(self.left_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                        else:
                            if (self.right_side >= 0):
                                #print("2")
                                for command in [0, 4]:
                                    self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                            else:
                                for command in [1, 5]:
                                    #print("22")
                                    true_right = abs(self.right_side)
                                    self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                                    #print(' out 22')
                    time.sleep(1)
                    print(f"3robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                    pose_msg = Pose()
                    pose_msg.position.x = float(self.pose_x)
                    pose_msg.position.y = float(self.pose_y)
                    self.pose_callback(pose_msg)
            print(self.absolute < self.threshold)
            if ((self.absolute < self.threshold)):
                print(f"Goal reached!!!!!!!!!!, x pos: {self.pose_x}, y pos: {self.pose_y}.")
                self.start = 1
                self.t = 0
                self.x = 1
                self.reach = 1
                reached = Pose()
                reached.position.x = 1000.0
                self.pub1.publish(reached)
                self.publish_goal = 1
                self.rotate_goal_true = 1
                #self.rotate_bot()

def stop_fun(self):
    print("entering stop_fun")
    while True:
        for address in [128,129]:
            for command in [0,1,4,5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)

def main(args=None):
    rclpy.init(args=args)
    check = Check()
    try:
        rclpy.spin(check)
    except KeyboardInterrupt:
        check.stop_motors()
        print("Keyboard Interrupt detected. Stopping motors.")
    finally:
        check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
THIS ONE IS GOOD
#Shivam working June5th 11:30am

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
import serial
import time
from math import sqrt, atan2, sin, cos, degrees

# Initialize the serial connection
sabertooth_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)

class Check(Node):
    def __init__(self):
        super().__init__('direct_coordinate')
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

        self.main_left = 0
        self.main_right = 0
        self.i = 0 
        self.t = 0
        self.publish_goal = 0
        self.start = 0
        self.x = 1
        self.reach = 0

        self.threshold = 0.5
        #self.theta_error = 0.2
        self.absolute = 0.0
        self.new_y = 0.0
        self.goal_angle = 0.0
        self.theta_old = 0.0
        self.theta_new = 0.0
        self.theta_degree = 0.0
        self.rotate_goal_true = 0
        self.callback = 0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.kp = 0.0

        self.sub = self.create_subscription(Float32MultiArray, '/pwm_signals', self.pwm_callback, 10)
        self.sub1 = self.create_subscription(Pose, '/pose', self.goal_pos, 10)
        self.pub = self.create_publisher(Pose, '/self_pose', 10)
        self.pub1 = self.create_publisher(Pose, "/goal_reached", 10)

        self.timer = self.create_timer(0.1, self.command_send)

    def pwm_callback(self, msg):
        self.right_bot = msg.data[0]
        self.left_bot = msg.data[2]
        self.right_top = msg.data[4]
        self.left_top = msg.data[6]
        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")

    def calculate_checksum(self, address, command, value):
        return (address + command + int(value)) & 0b01111111

    def calculate_angle(self):
        self.theta_new = atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)
        print(self.theta_new)
        #if abs(self.theta_new - self.theta_old) > self.theta_error:
        if abs(self.theta_new - self.theta_old) > 0.0:
            self.theta_degree = degrees(self.theta_new) - degrees(self.theta_old)
            self.theta = self.theta_new - self.theta_old
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
            self.theta_old = self.theta_new
        else:
            self.theta_degree = 0.0
            self.theta = 0.0
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")

    def rotate_bot(self):
        if self.theta_degree != 0.0:
            print("Rotate_bot")
            rotation = Twist()
            #if self.theta_degree > 0.0:
            rotation.linear.x = 0.0
            rotation.angular.z = 10.0
            self.cmd_callback(rotation)
            time.sleep(abs((3*self.theta_degree)/90))
            print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
            #else:
            #    rotation.linear.x = 0.0
            #    rotation.angular.z = -10.0
            #    self.cmd_callback(rotation)
            #    time.sleep(abs((3*(self.theta_degree))/90))
            #    print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
        else:
            print("No rotation")
        self.right_bot = 0.0
        self.left_bot = 0.0
        self.right_top = 0.0
        self.left_top = 0.0

    def send_sabertooth_command(self, serial_port, address, command, value):
        checksum = self.calculate_checksum(address, command, value)
        packet = bytes([address, command, int(value), checksum])
        serial_port.write(packet)
        #print(f"Sent command to Sabertooth: Address: {address}, Command: {command}, Value: {int(value)}, Checksum: {checksum}")

    def stop_motors(self):
        """Send stop command to all motors."""
        for address in [128, 129]:
            for command in [0, 1, 4, 5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)
        print("Motors stopped.")

    def goal_pos(self, data):
        self.callback = 1
        self.goal_x = data.position.x
        self.goal_y = data.position.y
        print(f"goal_x: {self.goal_x}, goal_y: {self.goal_y}")
        self.start = 0
        self.reach = 0
        self.absolute_()
        self.calculate_angle()
        self.rotate_bot()
        self.stop_motors()
        self.command_send()

    def kp_(self):
        if self.start == 0:
            if abs(self.goal_x - self.pose_x) > 60:
                self.kp = 7
            elif 40 < abs(self.goal_x - self.pose_x) <= 60:
                self.kp = 6
            elif 20 < abs(self.goal_x - self.pose_x) <= 40:
                self.kp = 5
            elif 10 < abs(self.goal_x - self.pose_x) <= 20:
                self.kp = 3
            elif 3 < abs(self.goal_x - self.pose_x) <= 10:
                self.kp = 2
            else:
                self.kp = 1

    def absolute_(self):
        if self.start == 0:
            self.absolute = sqrt((self.goal_x - self.pose_x)**2 + (self.goal_y - self.pose_y)**2)
            print(f"absolute value: {self.absolute}")

    def command_send(self):
        if self.callback == 1:
            if self.reach == 0:
                self.absolute_()
                if self.start == 0:
                    self.kp_()
                    print(f"ROBOT pose (x,y): ({self.pose_x},{self.pose_y}) and kp: {self.kp}")
                    if self.absolute > self.threshold:
                        goal_data = Twist()
                        goal_data.linear.x = self.kp * 10.0
                        goal_data.angular.z = 0.0
                        print(f"speed: {goal_data.linear.x}")
                        print("#########################################")
                        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")
                        print(self.right_bot != 0 and self.right_top != 0 and self.left_bot != 0 and self.left_top != 0)
                        if self.right_bot != 0 and self.right_top != 0 and self.left_bot != 0 and self.left_top != 0:
                            self.pose_x = self.pose_x + (goal_data.linear.x) * cos(self.theta_new) / 10
                            self.pose_y = self.pose_y + (goal_data.linear.x) * sin(self.theta_new) / 10
                        else:
                            self.pose_x = self.pose_x
                            self.pose_y = self.pose_y
                        self.cmd_callback(goal_data)
                    else:
                        self.callback = 0
                        goal_data1 = Twist()
                        goal_data1.linear.x = 0.0
                        goal_data1.angular.z = 0.0
                        self.cmd_callback(goal_data1)
                        self.stop_motors()
                        goal_reached = Pose()
                        goal_reached.position.x = 1000.0
                        self.pub1.publish(goal_reached)

    def pose_callback(self, data):
        self.pub.publish(data)

    def cmd_callback(self, msg):
        if self.start == 0:
            print("cmd received!!!")
            if msg.linear.x != 0.0 and msg.angular.z == 0.0:
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                for address in [128, 129]:
                    if address == 129:
                        if self.left_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            for command in [1, 5]:
                                true_left = abs(self.left_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if self.right_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            for command in [1, 5]:
                                true_right = abs(self.right_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                time.sleep(1)
                print(f"robot1 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)
            elif msg.linear.x == 0.0 and msg.angular.z != 0.0:
                for address in [128, 129]:
                    if address == 129:
                        if msg.angular.z > 0:
                            self.left_side = int(msg.angular.z)
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            self.left_side = int(abs(msg.angular.z))
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                    else:
                        if msg.angular.z > 0:
                            self.right_side = int(abs(msg.angular.z))
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            self.right_side = int(msg.angular.z)
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                time.sleep(1)
                print(f"robot2 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    check = Check()
    try:
        rclpy.spin(check)
    except KeyboardInterrupt:
        check.stop_motors()
        print("Keyboard Interrupt detected. Stopping motors.")
    finally:
        check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

# shivam 15th nov
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
import serial
import time
from math import sqrt, atan2, sin, cos, degrees

# Initialize the serial connection
# This is for Xavier (uncomment this for direct Xavier run)
#sabertooth_serial = serial.Serial('/dev/ttyTHS0', 9600, timeout=1)
#This is for pi
sabertooth_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)
# This is for laptop test version, comment out during Xavier run
#sabertooth_serial = serial.Serial('/dev/pts/2', 9600, timeout=1)

class Check(Node):
    def __init__(self):
        super().__init__('direct_coordinate')
        #considering the rover initial pos as (0,0,0) for coordinate control
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

        # consifering the rover main_left and main_right wheel speed as 0.
        self.main_left = 0
        self.main_right = 0
        self.i = 0 
        self.t = 0
        self.publish_goal = 0
        self.start = 0
        self.x = 1
        self.reach = 0

        self.threshold = 0.6
        #self.theta_error = 0.2
        self.absolute = 0.0
        self.new_y = 0.0

        # considering the goal angle to be 0.
        self.goal_angle = 0.0
        # considering the angle of the rover, it need to move during the new coordinates 
        self.theta_old = 0.0
        # updating the new angle of the rover to move in that dir 
        self.theta_new = 0.0
        #intial 0 
        self.theta_degree = 0.0
        self.odo_theta = 0.0
        self.rotate_goal_true = 0
        self.callback = 0
        self.Left_dis = 0.0
        self.Right_dis = 0.0
        
        # considering dis covered in one complete rotation
        self.pi = 3.14
        self.radius = 7.5 #cm
        self.Wheel_base = 26 #cm

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.kp = 0.0

        self.sub = self.create_subscription(Float32MultiArray, '/pwm_signals', self.pwm_callback, 10)
        self.sub1 = self.create_subscription(Pose, '/pose', self.goal_pos, 10)
        self.pub = self.create_publisher(Pose, '/self_pose', 10)
        self.pub1 = self.create_publisher(Pose, "/goal_reached", 10)

        self.timer = self.create_timer(0.1, self.command_send)

    def pwm_callback(self, msg):
        self.right_bot = msg.data[0]
        self.left_bot = msg.data[1]
        self.right_top = msg.data[2]
        self.left_top = msg.data[3]
        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")
        #self.Left_dis = ((self.left_bot + self.left_top)/2)/620
        #self.Right_dis = ((self.right_bot + self.right_top)/2)/620

    def calculate_checksum(self, address, command, value):
        return (address + command + int(value)) & 0b01111111

    def calculate_angle(self):
        self.theta_new = atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)
        print(self.theta_new)
        #if abs(self.theta_new - self.theta_old) > self.theta_error:
        if abs(self.theta_new - self.theta_old) > 0.0:
            self.theta_degree = degrees(self.theta_new) - degrees(self.theta_old)
            self.theta = self.theta_new - self.theta_old
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
            self.theta_old = self.theta_new
        else:
            self.theta_degree = 0.0
            self.theta = 0.0
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")

    def rotate_bot(self):
        if self.theta_degree != 0.0:
            print("Rotate_bot")
            rotation = Twist()
            #if self.theta_degree > 0.0:
            rotation.linear.x = 0.0
            rotation.angular.z = 10.0
            self.cmd_callback(rotation)
            time.sleep(abs((3*self.theta_degree)/90))
            print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
            #else:
            #    rotation.linear.x = 0.0
            #    rotation.angular.z = -10.0
            #    self.cmd_callback(rotation)
            #    time.sleep(abs((3*(self.theta_degree))/90))
            #    print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
        else:
            print("No rotation")
        self.right_bot = 0.0
        self.left_bot = 0.0
        self.right_top = 0.0
        self.left_top = 0.0

    def send_sabertooth_command(self, serial_port, address, command, value):
        checksum = self.calculate_checksum(address, command, value)
        packet = bytes([address, command, int(value), checksum])
        serial_port.write(packet)
        #print(f"Sent command to Sabertooth: Address: {address}, Command: {command}, Value: {int(value)}, Checksum: {checksum}")

    def stop_motors(self):
        """Send stop command to all motors."""
        for address in [128, 129]:
            for command in [0, 1, 4, 5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)
        print("Motors stopped.")

    def goal_pos(self, data):
        self.callback = 1
        self.goal_x = data.position.x
        self.goal_y = data.position.y
        print(f"goal_x: {self.goal_x}, goal_y: {self.goal_y}")
        self.start = 0
        self.reach = 0
        self.absolute_()
        self.calculate_angle()
        self.rotate_bot()
        self.stop_motors()
        self.command_send()

    def kp_(self):
        if self.start == 0:
            if abs(self.goal_x - self.pose_x) > 60:
                self.kp = 7
            elif 40 < abs(self.goal_x - self.pose_x) <= 60:
                self.kp = 6
            elif 20 < abs(self.goal_x - self.pose_x) <= 40:
                self.kp = 5
            elif 10 < abs(self.goal_x - self.pose_x) <= 20:
                self.kp = 3
            elif 5 < abs(self.goal_x - self.pose_x) <= 10:
                self.kp = 2
            elif 3 < abs(self.goal_x - self.pose_x) <= 5:
                self.kp = 2
            elif 2 < abs(self.goal_x - self.pose_x) <=3:
                self.kp = 1.5
            elif 0 < abs(self.goal_x - self.pose_x) <=2:
                self.kp = 1
            #else:
            #    self.start = 1
            #    self.callback = 0
            #    goal_data1 = Twist()
            #    goal_data1.linear.x = 0.0
            #    goal_data1.angular.z = 0.0
            #    self.cmd_callback(goal_data1)
            #    self.stop_motors()
            #    goal_reached = Pose()
            #    goal_reached.position.x = 1000.0
            #    self.pub1.publish(goal_reached)
       


    def absolute_(self):
        if self.start == 0:
            self.absolute = sqrt((self.goal_x - self.pose_x)**2 + (self.goal_y - self.pose_y)**2)
            print(f"absolute value: {self.absolute}")

    def command_send(self):
        if self.callback == 1:
            if self.reach == 0:
                self.absolute_()
                if self.start == 0:
                    self.kp_()
                    print(f"ROBOT pose (x,y): ({self.pose_x},{self.pose_y}) and kp: {self.kp}")
                    if self.absolute > self.threshold:
                        goal_data = Twist()
                        goal_data.linear.x = self.kp * 10.0
                        goal_data.angular.z = 0.0
                        print(f"speed: {goal_data.linear.x}")
                        print("#########################################")
                        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")
                        self.pose_x = self.pose_x + (((self.right_bot + self.right_top + self.left_bot + self.left_top)/4)/620)*2*self.pi*self.radius/10
                        self.pose_y = self.pose_y + (((self.right_bot + self.right_top + self.left_bot + self.left_top)/4)/620)*2*self.pi*self.radius/10
                        #self.pose_y = self.pose_y + (((self.right_bot + self.right_top + self.left_bot + self.left_top)/4)/620)**2*self.pi*self.radius/10
                        self.Left_dis = self.Left_dis + (((self.left_bot + self.left_top)/2)/620)*2*self.pi*self.radius/10
                        self.Right_dis = self.Right_dis + (((self.right_bot + self.right_top)/2)/620)*2*self.pi*self.radius/10
                        self.Total_dis = (self.Left_dis +self.Right_dis)/2
                        self.Theta_odo = (self.Left_dis - self.Right_dis)/self.Wheel_base
                        if (self.Theta_odo < 0.0):
                            print(f"robot leaning Left. Left dis{self.Left_dis}. Right wheel moving fast Right dis:{self.Right_dis}.")
                        elif (self.Theta_odo>0.0):
                            print(f"robot is leaning Right with speed {self.Right_dis}. Left wheel is fast with speed {self.Right_dis}.")
                        else:
                            print("Both moving same spped.")
                        self.cmd_callback(goal_data)
                        print(f"total dis: {self.Total_dis}, with odom_theta: {self.Theta_odo}")
                    else:
                        self.callback = 0
                        goal_data1 = Twist()
                        goal_data1.linear.x = 0.0
                        goal_data1.angular.z = 0.0
                        self.cmd_callback(goal_data1)
                        self.stop_motors()
                        goal_reached = Pose()
                        goal_reached.position.x = 1000.0
                        self.pub1.publish(goal_reached)

    def pose_callback(self, data):
        self.pub.publish(data)

    def cmd_callback(self, msg):
        if self.start == 0:
            print("cmd received!!!")
            if msg.linear.x != 0.0 and msg.angular.z == 0.0:
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                for address in [128, 129]:
                    if address == 129:
                        if self.left_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            for command in [1, 5]:
                                true_left = abs(self.left_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if self.right_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            for command in [1, 5]:
                                true_right = abs(self.right_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                time.sleep(1)
                print(f"robot1 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)
            elif msg.linear.x == 0.0 and msg.angular.z != 0.0:
                for address in [128, 129]:
                    if address == 129:
                        if msg.angular.z > 0:
                            self.left_side = int(msg.angular.z)
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            self.left_side = int(abs(msg.angular.z))
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                    else:
                        if msg.angular.z > 0:
                            self.right_side = int(abs(msg.angular.z))
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            self.right_side = int(msg.angular.z)
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                time.sleep(1)
                print(f"robot2 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    check = Check()
    try:
        rclpy.spin(check)
    except KeyboardInterrupt:
        check.stop_motors()
        print("Keyboard Interrupt detected. Stopping motors.")
    finally:
        check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


#shivam new 16th nov
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
import serial
import time
from math import sqrt, atan2, sin, cos, degrees

# Initialize the serial connection
# This is for Xavier (uncomment this for direct Xavier run)
#sabertooth_serial = serial.Serial('/dev/ttyTHS0', 9600, timeout=1)
# This is for pi (uncomment this for direct pi run)
sabertooth_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)
# This is for laptop test version, comment out during Xavier run
#sabertooth_serial = serial.Serial('/dev/pts/2', 9600, timeout=1)

class Check(Node):
    def __init__(self):
        super().__init__('direct_coordinate')
        #considering the rover initial pos as (0,0,0) for coordinate control
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

        # consifering the rover main_left and main_right wheel speed as 0.
        self.main_left = 0
        self.main_right = 0
        self.i = 0
        self.t = 0
        self.publish_goal = 0
        self.start = 0
        self.x = 1
        self.reach = 0

        self.threshold = 0.5
        #self.theta_error = 0.2
        self.absolute = 0.0
        self.new_y = 0.0

        # considering the goal angle to be 0.
        self.goal_angle = 0.0
        # considering the angle of the rover, it need to move during the new coordinates
        self.theta_old = 0.0
        # updating the new angle of the rover to move in that dir
        self.theta_new = 0.0
        #intial 0
        self.theta_degree = 0.0
        self.odo_theta = 0.0
        self.rotate_goal_true = 0
        self.callback = 0
        self.Left_dis = 0.0
        self.Right_dis = 0.0

        # considering dis covered in one complete rotation
        self.pi = 3.14
        self.radius = 7.5 #cm
        self.Wheel_base = 26 #cm

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.kp = 0.0

        self.sub = self.create_subscription(Float32MultiArray, '/pwm_signals', self.pwm_callback, 10)
        self.sub1 = self.create_subscription(Pose, '/pose', self.goal_pos, 10)
        self.pub = self.create_publisher(Pose, '/self_pose', 10)
        self.pub1 = self.create_publisher(Pose, "/goal_reached", 10)

        self.timer = self.create_timer(0.1, self.command_send)

    def pwm_callback(self, msg):
        self.right_bot = msg.data[0]
        self.left_bot = msg.data[1]
        self.right_top = msg.data[2]
        self.left_top = msg.data[3]
        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")
        #self.Left_dis = ((self.left_bot + self.left_top)/2)/620
        #self.Right_dis = ((self.right_bot + self.right_top)/2)/620

    def calculate_checksum(self, address, command, value):
        return (address + command + int(value)) & 0b01111111

    def calculate_angle(self):
        self.theta_new = atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)
        print(self.theta_new)
        #if abs(self.theta_new - self.theta_old) > self.theta_error:
        if abs(self.theta_new - self.theta_old) > 0.0:
            self.theta_degree = degrees(self.theta_new) - degrees(self.theta_old)
            self.theta = self.theta_new - self.theta_old
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
            self.theta_old = self.theta_new
        else:
            self.theta_degree = 0.0
            self.theta = 0.0
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")

    def rotate_bot(self):
        if self.theta_degree != 0.0:
            print("Rotate_bot")
            rotation = Twist()
            #if self.theta_degree > 0.0:
            rotation.linear.x = 0.0
            rotation.angular.z = 10.0
            self.cmd_callback(rotation)
            time.sleep(abs((3*self.theta_degree)/90))
            print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
            #else:
            #    rotation.linear.x = 0.0
            #    rotation.angular.z = -10.0
            #    self.cmd_callback(rotation)
            #    time.sleep(abs((3*(self.theta_degree))/90))
            #    print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
        else:
            print("No rotation")
        self.right_bot = 0.0
        self.left_bot = 0.0
        self.right_top = 0.0
        self.left_top = 0.0

    def send_sabertooth_command(self, serial_port, address, command, value):
        checksum = self.calculate_checksum(address, command, value)
        packet = bytes([address, command, int(value), checksum])
        serial_port.write(packet)
        #print(f"Sent command to Sabertooth: Address: {address}, Command: {command}, Value: {int(value)}, Checksum: {checksum}")

    def stop_motors(self):
        """Send stop command to all motors."""
        for address in [128, 129]:
            for command in [0, 1, 4, 5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)
        print("Motors stopped.")

    def goal_pos(self, data):
        self.callback = 1
        self.goal_x = data.position.x
        self.goal_y = data.position.y
        print(f"goal_x: {self.goal_x}, goal_y: {self.goal_y}")
        self.start = 0
        self.reach = 0
        self.absolute_()
        self.calculate_angle()
        self.rotate_bot()
        self.stop_motors()
        self.command_send()

    def kp_(self):
        if self.start == 0:
            if (self.goal_x - self.pose_x) > 60:
                self.kp = 7
            elif 40 < (self.goal_x - self.pose_x) <= 60:
                self.kp = 6
            elif 20 < (self.goal_x - self.pose_x) <= 40:
                self.kp = 5
            elif 10 < (self.goal_x - self.pose_x) <= 20:
                self.kp = 3
            elif 3 < (self.goal_x - self.pose_x) <= 10:
                self.kp = 2
            elif 0 < (self.goal_x - self.pose_x) <= 3:
                self.kp = 1
            else:
                self.stop_motors()
                self.start = 1
                self.callback = 0
                goal_data1 = Twist()
                goal_data1.linear.x = 0.0
                goal_data1.angular.z = 0.0
                self.cmd_callback(goal_data1)
                self.stop_motors()
                goal_reached = Pose()
                goal_reached.position.x = 1000.0
                self.pub1.publish(goal_reached)

    def absolute_(self):
        if self.start == 0:
            self.absolute = sqrt((self.goal_x - self.pose_x)**2 + (self.goal_y - self.pose_y)**2)
            print(f"absolute value: {self.absolute}")

    def command_send(self):
        if self.callback == 1:
            if self.reach == 0:
                self.absolute_()
                if self.start == 0:
                    self.kp_()
                    print(f"ROBOT pose (x,y): ({self.pose_x},{self.pose_y}) and kp: {self.kp}")
                    if self.absolute > self.threshold:
                        goal_data = Twist()
                        goal_data.linear.x = self.kp * 10.0
                        goal_data.angular.z = 0.0
                        print(f"speed: {goal_data.linear.x}")
                        print("#########################################")
                        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")
                        self.pose_x = self.pose_x + (((self.right_bot + self.right_top + self.left_bot + self.left_top)/4)/620)*2*self.pi*self.radius/10
                        self.pose_y = self.pose_y + (((self.right_bot + self.right_top + self.left_bot + self.left_top)/4)/620)**2*self.pi*self.radius/10
                        self.Left_dis = self.Left_dis + (((self.left_bot + self.left_top)/2)/620)*2*self.pi*self.radius/10
                        self.Right_dis = self.Right_dis + (((self.right_bot + self.right_top)/2)/620)*2*self.pi*self.radius/10
                        self.Total_dis = (self.Left_dis +self.Right_dis)/2
                        self.Theta_odo = (self.Left_dis - self.Right_dis)/self.Wheel_base
                        if (self.Theta_odo < 0.0):
                            print(f"robot leaning Left. Left dis{self.Left_dis}. Right wheel moving fast Right dis:{self.Right_dis}.")
                        elif (self.Theta_odo>0.0):
                            print(f"robot is leaning Right with speed {self.Right_dis}. Left wheel is fast with speed {self.Right_dis}.")
                        else:
                            print("Both moving same spped.")
                        self.cmd_callback(goal_data)
                        print(f"total dis: {self.Total_dis}, with odom_theta: {self.Theta_odo}")
                    else:
                        self.callback = 0
                        goal_data1 = Twist()
                        goal_data1.linear.x = 0.0
                        goal_data1.angular.z = 0.0
                        self.cmd_callback(goal_data1)
                        self.stop_motors()
                        goal_reached = Pose()
                        goal_reached.position.x = 1000.0
                        self.pub1.publish(goal_reached)

    def pose_callback(self, data):
        self.pub.publish(data)

    def cmd_callback(self, msg):
        if self.start == 0:
            print("cmd received!!!")
            if msg.linear.x != 0.0 and msg.angular.z == 0.0:
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                for address in [128, 129]:
                    if address == 129:
                        if self.left_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            for command in [1, 5]:
                                true_left = abs(self.left_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if self.right_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            for command in [1, 5]:
                                true_right = abs(self.right_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                time.sleep(1)
                print(f"robot1 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)
            elif msg.linear.x == 0.0 and msg.angular.z != 0.0:
                for address in [128, 129]:
                    if address == 129:
                        if msg.angular.z > 0:
                            self.left_side = int(msg.angular.z)
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            self.left_side = int(abs(msg.angular.z))
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                    else:
                        if msg.angular.z > 0:
                            self.right_side = int(abs(msg.angular.z))
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            self.right_side = int(msg.angular.z)
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                time.sleep(1)
                print(f"robot2 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    check = Check()
    try:
        rclpy.spin(check)
    except KeyboardInterrupt:
        check.stop_motors()
        print("Keyboard Interrupt detected. Stopping motors.")
    finally:
        check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


#this one is good and if not go to 15th nov
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
import serial
import time
from math import sqrt, atan2, sin, cos, degrees

# Initialize the serial connection
sabertooth_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)

class Check(Node):
    def __init__(self):
        super().__init__('direct_coordinate')
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

        self.main_left = 0
        self.main_right = 0
        self.i = 0
        self.t = 0
        self.publish_goal = 0
        self.start = 0
        self.x = 1
        self.reach = 0

        self.threshold = 0.5
        #self.theta_error = 0.2
        self.absolute = 0.0
        self.new_y = 0.0
        self.goal_angle = 0.0
        self.theta_old = 0.0
        self.theta_new = 0.0
        self.theta_degree = 0.0
        self.rotate_goal_true = 0
        self.callback = 0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.kp = 0.0

        self.sub = self.create_subscription(Float32MultiArray, '/pwm_signals', self.pwm_callback, 10)
        self.sub1 = self.create_subscription(Pose, '/pose', self.goal_pos, 10)
        self.pub = self.create_publisher(Pose, '/self_pose', 10)
        self.pub1 = self.create_publisher(Pose, "/goal_reached", 10)

        self.timer = self.create_timer(0.1, self.command_send)

    def pwm_callback(self, msg):
        self.right_bot = msg.data[0]
        self.left_bot = msg.data[2]
        self.right_top = msg.data[4]
        self.left_top = msg.data[6]
        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")

    def calculate_checksum(self, address, command, value):
        return (address + command + int(value)) & 0b01111111

    def calculate_angle(self):
        self.theta_new = atan2(self.goal_y - self.pose_y, self.goal_x - self.pose_x)
        print(self.theta_new)
        #if abs(self.theta_new - self.theta_old) > self.theta_error:
        if abs(self.theta_new - self.theta_old) > 0.0:
            self.theta_degree = degrees(self.theta_new) - degrees(self.theta_old)
            self.theta = self.theta_new - self.theta_old
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")
            self.theta_old = self.theta_new
        else:
            self.theta_degree = 0.0
            self.theta = 0.0
            print(f" Old theta: {degrees(self.theta_old)}, New theta: {degrees(self.theta_new)}, Rotating radian: {self.theta}, Rotating degree: {self.theta_degree}")

    def rotate_bot(self):
        if self.theta_degree != 0.0:
            print("Rotate_bot")
            rotation = Twist()
            #if self.theta_degree > 0.0:
            rotation.linear.x = 0.0
            rotation.angular.z = 10.0
            self.cmd_callback(rotation)
            time.sleep(abs((3*self.theta_degree)/90))
            print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
            #else:
            #    rotation.linear.x = 0.0
            #    rotation.angular.z = -10.0
            #    self.cmd_callback(rotation)
            #    time.sleep(abs((3*(self.theta_degree))/90))
            #    print(f"time.sleep: {abs((3*self.theta_degree)/90)}")
        else:
            print("No rotation")
        self.right_bot = 0.0
        self.left_bot = 0.0
        self.right_top = 0.0
        self.left_top = 0.0

    def send_sabertooth_command(self, serial_port, address, command, value):
        checksum = self.calculate_checksum(address, command, value)
        packet = bytes([address, command, int(value), checksum])
        serial_port.write(packet)
        #print(f"Sent command to Sabertooth: Address: {address}, Command: {command}, Value: {int(value)}, Checksum: {checksum}")

    def stop_motors(self):
        """Send stop command to all motors."""
        for address in [128, 129]:
            for command in [0, 1, 4, 5]:
                self.send_sabertooth_command(sabertooth_serial, address, command, 0)
        print("Motors stopped.")

    def goal_pos(self, data):
        self.callback = 1
        self.goal_x = data.position.x
        self.goal_y = data.position.y
        print(f"goal_x: {self.goal_x}, goal_y: {self.goal_y}")
        self.start = 0
        self.reach = 0
        self.absolute_()
        self.calculate_angle()
        self.rotate_bot()
        self.stop_motors()
        self.command_send()

    def kp_(self):
        if self.start == 0:
            if abs(self.goal_x - self.pose_x) > 60:
                self.kp = 7
            elif 40 < abs(self.goal_x - self.pose_x) <= 60:
                self.kp = 6
            elif 20 < abs(self.goal_x - self.pose_x) <= 40:
                self.kp = 5
            elif 10 < abs(self.goal_x - self.pose_x) <= 20:
                self.kp = 3
            elif 3 < abs(self.goal_x - self.pose_x) <= 10:
                self.kp = 2
            else:
                self.kp = 1

    def absolute_(self):
        if self.start == 0:
            self.absolute = sqrt((self.goal_x - self.pose_x)**2 + (self.goal_y - self.pose_y)**2)
            print(f"absolute value: {self.absolute}")

    def command_send(self):
        if self.callback == 1:
            if self.reach == 0:
                self.absolute_()
                if self.start == 0:
                    self.kp_()
                    print(f"ROBOT pose (x,y): ({self.pose_x},{self.pose_y}) and kp: {self.kp}")
                    if self.absolute > self.threshold:
                        goal_data = Twist()
                        goal_data.linear.x = self.kp * 10.0
                        goal_data.angular.z = 0.0
                        print(f"speed: {goal_data.linear.x}")
                        print("#########################################")
                        print(f"({self.right_bot:.2f},{self.left_bot:.2f},{self.right_top:.2f},{self.left_top:.2f})")
                        print(self.right_bot != 0 and self.right_top != 0 and self.left_bot != 0 and self.left_top != 0)
                        if self.right_bot != 0 and self.right_top != 0 and self.left_bot != 0 and self.left_top != 0:
                            self.pose_x = self.pose_x + (goal_data.linear.x) * cos(self.theta_new) / 10
                            self.pose_y = self.pose_y + (goal_data.linear.x) * sin(self.theta_new) / 10
                        else:
                            self.pose_x = self.pose_x
                            self.pose_y = self.pose_y
                        self.cmd_callback(goal_data)
                    else:
                        self.callback = 0
                        goal_data1 = Twist()
                        goal_data1.linear.x = 0.0
                        goal_data1.angular.z = 0.0
                        self.cmd_callback(goal_data1)
                        self.stop_motors()
                        goal_reached = Pose()
                        goal_reached.position.x = 1000.0
                        self.pub1.publish(goal_reached)

    def pose_callback(self, data):
        self.pub.publish(data)

    def cmd_callback(self, msg):
        if self.start == 0:
            print("cmd received!!!")
            if msg.linear.x != 0.0 and msg.angular.z == 0.0:
                self.left_side = int(msg.linear.x)
                self.right_side = int(msg.linear.x)
                self.main_left = int(self.main_left + self.left_side)
                self.main_right = int(self.main_right + self.right_side)
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                for address in [128, 129]:
                    if address == 129:
                        if self.left_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            for command in [1, 5]:
                                true_left = abs(self.left_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_left)
                    else:
                        if self.right_side >= 0:
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            for command in [1, 5]:
                                true_right = abs(self.right_side)
                                self.send_sabertooth_command(sabertooth_serial, address, command, true_right)
                time.sleep(1)
                print(f"robot1 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)
            elif msg.linear.x == 0.0 and msg.angular.z != 0.0:
                for address in [128, 129]:
                    if address == 129:
                        if msg.angular.z > 0:
                            self.left_side = int(msg.angular.z)
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                        else:
                            self.left_side = int(abs(msg.angular.z))
                            self.main_left = int(self.main_left + self.left_side)
                            self.main_left = max(min(self.main_left, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.left_side)
                    else:
                        if msg.angular.z > 0:
                            self.right_side = int(abs(msg.angular.z))
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [1, 5]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                        else:
                            self.right_side = int(msg.angular.z)
                            self.main_right = int(self.main_right + self.right_side)
                            self.main_right = max(min(self.main_right, 127), -127)
                            for command in [0, 4]:
                                self.send_sabertooth_command(sabertooth_serial, address, command, self.right_side)
                time.sleep(1)
                print(f"robot2 - x pos: {self.pose_x}, y pos: {self.pose_y}")
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    check = Check()
    try:
        rclpy.spin(check)
    except KeyboardInterrupt:
        check.stop_motors()
        print("Keyboard Interrupt detected. Stopping motors.")
    finally:
        check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

