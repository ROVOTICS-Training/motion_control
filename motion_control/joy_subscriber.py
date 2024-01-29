#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64
class JoySubscriber(Node):
    def __init__(self):
        super().__init__("joy_subscriber")
        self.log = self.get_logger()
        self.log.info("This is the joy subscriber node. ")
        self.subscriber = self.create_subscription(Joy, "joy", self.receive_callback, 10)
    def receive_callback(self, msg: Joy):
        slow_down = bool(msg.buttons[len(msg.buttons)-1])


def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()