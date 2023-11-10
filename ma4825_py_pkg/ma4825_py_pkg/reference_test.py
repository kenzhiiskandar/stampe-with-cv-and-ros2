"This program testing if ROS2 works"

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node): #MyNode is the class name

    def __init__(self):
        super().__init__("test_node") #py_test is the node name
        self.counter_= 0 #initialize variable to 0
        self.create_timer(0.5,self.timer_callback) #to create a timer with 0.5s betwwen each callback
        self.get_logger().info("Hello ROS2") #to print something

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))

def main(args=None): #This main function will always be the same
    rclpy.init(args=args) #to initialize ros2 communication
    node = MyNode()
    rclpy.spin(node) #to keep the program run
    rclpy.shutdown()


if __name__=="__main__":
    main()
