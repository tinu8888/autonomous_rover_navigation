import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Motion(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.main_left = 0.0
        self.main_right = 0.0
        self.left_side = 0.0
        self.right_side = 0.0

        self.sub = self.create_subscription(Twist, '/command', self.cmd_callback, 10)

    def cmd_callback(self, msg):
        print("cmd received!!!")
        if (msg.linear.x != 0.0 and msg.angular.z == 0.0):
            self.left_side = msg.linear.x
            self.right_side = msg.linear.x
            self.main_left = self.main_left + self.left_side
            self.main_right = self.main_right + self.right_side
            self.main_left = max(min(self.main_left, 127), -127)
            self.main_right = max(min(self.main_right, 127), -127)
            print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")
        
        elif (msg.linear.x == 0.0 and msg.angular.z != 0.0):
            if(msg.angular.z > 0.0):
                self.left_side = abs(msg.angular.z)
                self.right_side = -abs(msg.angular.z)
                self.main_left = self.main_left + self.left_side
                self.main_right = self.main_right + self.right_side
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")
            
            else :
                self.left_side = -abs(msg.angular.z)
                self.right_side = abs(msg.angular.z)
                self.main_left = self.main_left + self.left_side
                self.main_right = self.main_right + self.right_side
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")

        else :
            if(msg.angular.z > 0.0):
                self.main_left = msg.linear.x + msg.angular.z
                self.main_right = msg.linear.x - msg.angular.z
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")
            
            else:
                self.main_left = msg.linear.x - msg.angular.z
                self.main_right = msg.linear.x + msg.angular.z
                self.main_left = max(min(self.main_left, 127), -127)
                self.main_right = max(min(self.main_right, 127), -127)
                print(f"main_left: {self.main_left}, main_right: {self.main_right}, left_side: {self.left_side}, right_side: {self.right_side} .")



def main(args=None):
    rclpy.init(args=args)
    motion_node = Motion()
    rclpy.spin(motion_node)
    motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()