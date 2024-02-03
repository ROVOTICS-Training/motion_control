#! usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from rclpy.parameter import Parameter

class VectorConversion(Node):
    def __init__(self):
        super().__init__("vector_conversion")
        self.log = self.get_logger()
        self.log.info("This is the vector conversion node. ")
        self.joy_subscriber = self.create_subscription(Joy, "joy", self.input_callback, 10)
        self.vector_publisher = self.create_publisher(Twist, "converted_vectors", 10)
        self.last_button = False
        self.thrusters_status = True

        sensitivity_range = FloatingPointRange()
        sensitivity_range.from_value = 0.0
        sensitivity_range.to_value = 1.0
        sensitivity_range.step = 0.01
        sensitivity_slider = ParameterDescriptor(floating_point_range = [sensitivity_range])

        self.declare_parameter("linear_x_sensitivity", 0.5, sensitivity_slider)
        self.declare_parameter("linear_y_sensitivity", 0.5, sensitivity_slider)
        self.declare_parameter("linear_z_sensitivity", 0.5, sensitivity_slider)
        self.declare_parameter("angular_x_sensitivity", 0.25, sensitivity_slider)
        self.declare_parameter("angular_z_sensitivity", 0.25, sensitivity_slider)
        self.declare_parameter("slow_down_sensitivity", 0.5, sensitivity_slider)

        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.type_ is Parameter.Type.DOUBLE and 0 < param.value <= 1:
                if param.name == "linear_x_sensitivity":
                    self.linear_x_sensitivity = param.value
                elif param.name == "linear_y_sensitivity":
                    self.linear_y_sensitivity = param.value
                elif param.name == "linear_z_sensitivity":
                    self.linear_z_sensitivity = param.value
                elif param.name == "angular_x_sensitivity":
                    self.angular_x_sensitivity = param.value
                elif param.name == "angular_z_sensitivity":
                    self.angular_z_sensitivity = param.value
                elif param.name == "slow_down_sensitivity":
                    self.slow_down_sensitivity = param.value
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)
        
    def input_callback(self, joystick: Joy):
        # Turn thrusters on and off
        if bool(joystick.buttons[11]) and not self.last_button:
            self.thrusters_status = not self.thrusters_status
        self.last_button = joystick.buttons[11]

        if self.thrusters_status:
            # Slow down check
            if bool(joystick.buttons[1]): # if slow down button is pressed
                slow_down = self.get_parameter("slow_down_sensitivity").value
            else:
                slow_down = 1
            # Make vectors
            vectors = Twist()

            vectors.linear.x = joystick.axes[0] * slow_down * self.get_parameter("linear_x_sensitivity").value # Side to Side
            vectors.linear.y = joystick.axes[1] * slow_down * self.get_parameter("linear_y_sensitivity").value # Forward and Backward
            vectors.linear.z = joystick.axes[3] * slow_down * self.get_parameter("linear_z_sensitivity").value # Up and Down

            vectors.angular.x = joystick.axes[4] * slow_down * self.get_parameter("angular_x_sensitivity").value # Rolling
            vectors.angular.z = joystick.axes[2] * slow_down * self.get_parameter("angular_z_sensitivity").value # Look Around
            self.vector_publisher.publish(vectors)
        else:
            self.vector_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VectorConversion()
    rclpy.spin(node)
    rclpy.shutdown()