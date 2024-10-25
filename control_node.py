import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import sys
import tty
import termios

class Control(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(Twist, '/command', 10)
        self.pub = self.create_publisher(Pose, '/pose', 10)
        self.twist_msg_ = Twist()
        print("Press 'w' to move forward, 's' to move backward, 'a' to turn right, 'd' to turn left and press 'q' to exit.")

    def publish_twist(self, linear_x, angular_z):
        self.twist_msg_.linear.x = linear_x
        self.twist_msg_.angular.z = angular_z
        self.publisher_.publish(self.twist_msg_)

def catch_button():
    f = sys.stdin.fileno()
    old_settings = termios.tcgetattr(f)
    try:
        tty.setraw(f)
        a = sys.stdin.read(1)
    finally:
        termios.tcsetattr(f, termios.TCSADRAIN, old_settings)
    return a


def main(args=None):
    rclpy.init(args=args)
    control_node = Control()
    try:
        while True:
            button = catch_button()
            if button == 'w' or button == 'W':
                control_node.publish_twist(1.0, 0.0)  
                print(f"You pressed {button}. Move forward.\n Press 'w' to move forward, 's' to move backward, 'a' to turn right, 'd' to turn left and press 'q' to exit.")
            elif button == 's' or button == 'S':
                control_node.publish_twist(-1.0, 0.0)  
                print(f"You pressed {button}. Move backward.\n Press 'w' to move forward, 's' to move backward, 'a' to turn right, 'd' to turn left and press 'q' to exit.")
            elif button == 'a' or button == 'A':
                control_node.publish_twist(0.0, 1.0)  
                print(f"You pressed {button}. Move left.\n Press 'w' to move forward, 's' to move backward, 'a' to turn right, 'd' to turn left and press 'q' to exit.")
            elif button == 'd' or button == 'D':
                control_node.publish_twist(0.0, -1.0)  
                print(f"You pressed {button}. Move right.\n Press 'w' to move forward, 's' to move backward, 'a' to turn right, 'd' to turn left and press 'q' to exit.")
            elif button == 'q' or button == 'Q':
                control_node.publish_twist(1.0, 1.0) 
                print("You exited by pressing 'q'. Good bye.")
                break 

    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()