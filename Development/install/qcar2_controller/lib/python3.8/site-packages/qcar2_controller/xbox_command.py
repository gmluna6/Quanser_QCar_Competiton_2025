#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from qcar2_interfaces.msg import MotorCommands, BooleanLeds

class JoyQCarController(Node):
    def __init__(self):
        super().__init__('joy_qcar_controller')

        # Publishers
        self.motor_pub = self.create_publisher(MotorCommands, 'qcar2_motor_speed_cmd', 10)
        self.led_pub = self.create_publisher(BooleanLeds, 'qcar2_led_cmd', 10)

        # Subscriber to the /joy topic
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.throttle = 0.0
        self.steering = 0.0

    def joy_callback(self, msg: Joy):
        try:
            # Logitech-style controller mapping (can be modified based on actual layout)
            LLA = 1.0 * msg.axes[0]  # Left stick X axis
            RT = (1.0 - msg.axes[5]) / 2.0  # Convert RT from [-1, 1] to [0, 1]
            LB = msg.buttons[4]  # LB
            A = msg.buttons[0]  # A

            led_values = [False] * 16

            if LB == 1:
                # Turn on front and rear LEDs
                for i in range(8, 14):
                    led_values[i] = True

                self.throttle = 0.3 * RT if RT > 0 else 0.0
                self.steering = 0.5 * LLA

                # Turn signals
                if self.steering > 0.01:
                    led_values[14] = True  # left front
                    led_values[6] = True   # left rear
                elif self.steering < -0.01:
                    led_values[15] = True  # right front
                    led_values[7] = True   # right rear

                if A == 1:
                    self.throttle = -self.throttle
                    led_values[4] = True  # left reverse
                    led_values[5] = True  # right reverse

            else:
                # Brake lights
                for i in range(4):
                    led_values[i] = True
                self.throttle = 0.0
                self.steering = 0.0

            # Publish motor commands
            motor_msg = MotorCommands()
            motor_msg.motor_names = ['steering_angle', 'motor_throttle']
            motor_msg.values = [self.steering, self.throttle]
            self.motor_pub.publish(motor_msg)

            # Publish LED commands
            led_msg = BooleanLeds()
            led_msg.led_names = [
                "left_outside_brake_light", "left_inside_brake_light", "right_inside_brake_light", "right_outside_brake_light",
                "left_reverse_light", "right_reverse_light", "left_rear_signal", "right_rear_signal",
                "left_outside_headlight", "left_middle_headlight", "left_inside_headlight", "right_inside_headlight",
                "right_middle_headlight", "right_outside_headlight", "left_front_signal", "right_front_signal"
            ]
            led_msg.values = led_values
            self.led_pub.publish(led_msg)

        except IndexError as e:
            self.get_logger().warn(f"Controller mapping error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyQCarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
