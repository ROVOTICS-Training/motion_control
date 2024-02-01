#! usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoySubscriber(Node):
    def __init__(self):
        super().__init__("vector_conversion")
        self.log = self.get_logger()
        self.log.info("This is the vector conversion node. ")
        self.joy_subscriber = self.create_subscription(Joy, "joy", self.receive_callback, 10)
        self.vector_publisher = self.create_publisher(Twist, "converted_vectors", 10)
        
    def receive_callback(self, joystick: Joy):
        slow_down = 1
        if bool(joystick.buttons[1]): # if slow down button is pressed
            slow_down = 0.5

        vectors = Twist()

        vectors.linear.x = joystick.axes[0] * slow_down # Side to Side
        vectors.linear.y = joystick.axes[1] * slow_down # Forward and Backward
        vectors.linear.z = joystick.axes[3] * slow_down # Up and Down

        vectors.angular.y = joystick.axes[4] * slow_down # Rolling
        vectors.angular.z = joystick.axes[2] * slow_down # Look Around
        self.vector_publisher.publish(vectors)

        
    

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()