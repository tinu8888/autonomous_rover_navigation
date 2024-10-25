import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import serial
import time
from math import sqrt

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
        super().__init__('check')
        self.main_left = 0
        self.main_right = 0
        self.left_side = 0.0
        self.right_side = 0.0
        self.i = 0 
        self.t =0
        self.publish_goal = 0
        self.start = 0
        self.x = 1

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.new_y = 0.0
        self.theta = 0.0

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
        self.command_send()

    def kp_(self):
        if (self.goal_x - self.pose_x != 0.0):
            if ((self.goal_x - self.pose_x)> 60):
                self.kp = 5.0
            elif ((self.goal_x - self.pose_x)> 40 and (self.goal_x - self.pose_x)  <= 60):
                self.kp = 1.0
            elif ((self.goal_x - self.pose_x)> 20 and (self.goal_x - self.pose_x)<= 40):
                self.kp = 0.8
            elif ((self.goal_x - self.pose_x)> 10 and (self.goal_x - self.pose_x) <= 20):
                self.kp = 0.4
            elif ((self.goal_x - self.pose_x) > 3 and (self.goal_x - self.pose_x)<= 10):
                self.kp = 0.2
            else:
                self.kp = 0.1
        else:
            if ((self.goal_y - self.new_y)> 60):
                self.kp = 5.0
            elif ((self.goal_y - self.new_y) > 40 and (self.goal_y - self.new_y) <= 60):
                self.kp = 1.0
            elif ((self.goal_y - self.new_y)> 20 and (self.goal_y - self.new_y)<= 40):
                self.kp = 0.8
            elif ((self.goal_y - self.new_y)> 10 and (self.goal_y - self.new_y) <= 20):
                self.kp = 0.4
            elif ((self.goal_y - self.new_y)> 3 and (self.goal_y - self.new_y) <= 10):
                self.kp = 0.2
            else:
                self.kp = 0.1
        
    def rotate_right(self):
        global NEW, NOW_ROTATE, MOVE
        
        print("rotating right")
        rotation = Twist()
        rotation.linear.x = 0.0
        rotation.angular.z = -10.0
        self.cmd_callback(rotation)
        if (self.new_y < self.goal_y):
            time.sleep(3)
        if (self.new_y > self.goal_y) :
            time.sleep(9)
        MOVE = False
        NOW_ROTATE = False
        NEW = True
        self.command_send()
        #print("now rotate False")

    def command_send(self):
        global NEXT
        while (self.pose_x != self.goal_x):
            print("******* self.start=0")
            self.start = 0
            global NOW_ROTATE
            NOW_ROTATE = True
            if (self.pose_x > self.goal_x):
                self.kp_()
                print(f"x pos: {self.pose_x} and kp: {self.kp}")
                goal_data = Twist()
                goal_data.linear.x = self.kp * -10.0
                goal_data.angular.z = 0.0
                self.cmd_callback(goal_data)

            elif (self.pose_x < self.goal_x):
                self.kp_()
                print(f"x pos: {self.pose_x} and kp: {self.kp}")
                goal_data = Twist()
                goal_data.linear.x = self.kp * 10.0
                goal_data.angular.z = 0.0
                self.cmd_callback(goal_data)

        #print("X done!!!!!!!!")
        print("Out of X pos++++++++++++++++++++++++++++++++++")
        if (self.i==0):
            self.i = 1
            self.rotate_right()
            
        print("entering in while")
        while (self.new_y != self.goal_y):
            #print("entered in new_y")
            self.t = 1
            global MOVE 
            MOVE = False
            NEXT = False
            if (self.new_y < self.goal_y):
                #print("Entered in self.new_y > self.goal_y")
                self.x = 0
                self.kp_()
                print(f"x pos: {self.new_y} and kp: {self.kp}")
                goal_data = Twist()
                goal_data.linear.x = self.kp * 10.0
                goal_data.angular.z = 0.0
                self.cmd_callback(goal_data)
                #MOVE = False

            elif (self.new_y > self.goal_y):
                #print("Entered in self.new_y < self.goal_y")
                self.x = 0
                self.kp_()
                print(f"x pos: {self.new_y} and kp: {self.kp}")
                goal_data = Twist()
                goal_data.linear.x = self.kp * -10.0
                goal_data.angular.z = 0.0
                self.cmd_callback(goal_data)
                NEXT = True
                #MOVE = False
                self.publish_goal = 0

        #print(f"Reached position: {self.pose_x}, {self.pose_y}")
        reached = Pose()
        if ((self.pose_x == self.goal_x) and (self.new_y == self.goal_y)):
            reached.position.x = 1000.0
            self.pub1.publish(reached)
            self.publish_goal = 1
            
        self.stop_motors()

    def pose_callback(self, data):
        self.pub.publish(data)

    def cmd_callback(self, msg):
        if(self.start == 0):
            print("cmd received!!!")
            print(f"self.new_y : {self.new_y}")
            if (msg.linear.x != 0.0 and msg.angular.z == 0.0):
                if self.x !=0 :
                    self.pose_x = self.pose_x + msg.linear.x
                if self.t != 0:
                    print("entered new")
                    self.new_y = self.new_y + msg.linear.x
                self.pose_y = self.new_y
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
                print(f"1robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                time.sleep(1)
                self.stop_motors()
                pose_msg = Pose()
                pose_msg.position.x = float(self.pose_x)
                pose_msg.position.y = float(self.pose_y)
                self.pose_callback(pose_msg)

            elif (msg.linear.x == 0.0 and msg.angular.z != 0.0):
                #self.pose_x = self.pose_x + msg.linear.x
                if NEW:
                    self.new_y = self.new_y + msg.linear.x
                #self.pose_y = self.new_y
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
                    print(f"3robot -  x pos: {self.pose_x }, y pos: {self.pose_y }")
                    time.sleep(1)
                    self.stop_motors()
                    pose_msg = Pose()
                    pose_msg.position.x = float(self.pose_x)
                    pose_msg.position.y = float(self.pose_y)
                    self.pose_callback(pose_msg)

            if (self.pose_x == self.goal_x and self.pose_y == self.goal_y):
                print(f"Goal reached!!!!!!!!!!, x pos: {self.pose_x}, y pos: {self.pose_y}.")
                self.start = 1
                self.t = 0
                self.x = 1
            else:
                self.command_send()

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
