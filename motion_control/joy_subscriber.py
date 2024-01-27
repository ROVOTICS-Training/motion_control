#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__("joy_subscriber")
        self.log = self.get_logger()
        self.log.info("This is the joy subscriber node. ")
        self.subscriber = self.create_subscription(
            Joy, "joy", self.receive_callback, 10)
    def receive_callback(self, msg: Joy):
        for button in msg.buttons:
            self.log.info(str(button))

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()