#Shivam working latest June 5th 11:30

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys
from math import sqrt, atan2, pi

class Swim_To_Goal(Node):
    pose_x=0.0
    pose_y=0.0
    theta=0.0
    
    def __init__(self):
        super().__init__('swim_to_goal')
        self.pose_publisher_ = self.create_publisher(Pose,"/pose", 10)
        self.sub = self.create_subscription(Pose, "/goal_reached", self.reached_callback, 10)
        self.set_goal()

    def reached_callback(self,data):
        if data.position.x == 1000.0 :
            self.set_goal()


    def set_goal(self):
        self.get_logger().info("Set")
        self.x2 = float(input("x_goal: "))
        self.y2 = float(input("y_goal: "))
        if self.x2>0.0:
            self.x2 = self.x2 % 50
        else:
            self.x2 = abs(self.x2) % 50
            self.x2 = -self.x2
        if self.y2>0.0:
            self.y2 = self.y2 % 50
        else:
            self.y2 = abs(self.y2) % 50
            self.y2 = -self.y2
        goal_data = Pose()
        goal_data.position.x = self.x2
        goal_data.position.y = self.y2
        print(goal_data)
        #goal_data.orientation.z = self.theta
        self.pose_publisher_.publish(goal_data)

def main(args=None):
    rclpy.init(args=args)
    swim_to_goal = Swim_To_Goal()
    rclpy.spin(swim_to_goal)
    swim_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
