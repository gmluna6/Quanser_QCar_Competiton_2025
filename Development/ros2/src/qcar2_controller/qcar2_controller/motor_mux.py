#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands
from std_msgs.msg import Bool

class MotorMux(Node):
    def __init__(self):
        super().__init__('motor_mux')
        self.stop_active = False
        self.redlight_active = False

        # Subscribers
        self.sub_nav2 = self.create_subscription(MotorCommands, '/qcar2_motor_speed_cmd/nav2', self.nav2_callback, 10)
        self.sub_lane = self.create_subscription( MotorCommands, '/qcar2_motor_speed_cmd/lane_following', self.lane_callback, 10)
        self.sub_stop_flag = self.create_subscription(Bool, '/stop_sign_detected', self.stop_flag_callback, 10)
        self.sub_redlight_flag = self.create_subscription(Bool, '/red_light_detected', self.redlight_flag_callback, 10)

        # Publisher
        self.pub_cmd = self.create_publisher( MotorCommands, '/qcar2_motor_speed_cmd', 10)

        # Last received messages
        self.last_nav2_msg = None
        self.last_lane_msg = None

        # Timer for publishing combined motor commands
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("motor_mux node with combined steering and red light handling started.")

    def stop_flag_callback(self, msg):
        self.stop_active = msg.data
        if self.stop_active:
            self.get_logger().info("STOP SIGN detected: stopping the car.")
        else:
            self.get_logger().info("STOP SIGN cleared: ready to resume.")

    def redlight_flag_callback(self, msg):
        self.redlight_active = msg.data
        if self.redlight_active:
            self.get_logger().info("RED LIGHT detected: stopping the car.")
        else:
            self.get_logger().info("RED LIGHT cleared: ready to resume.")

    def nav2_callback(self, msg):
        self.last_nav2_msg = msg

    def lane_callback(self, msg):
        self.last_lane_msg = msg

    def timer_callback(self):
        # Initialize defaults
        throttle = 0.0
        steering_nav = 0.0
        steering_lane = 0.0

        # If either stop sign OR red light is active → STOP the car
        if self.stop_active or self.redlight_active:
            steering_final = 0.0
            throttle = 0.0
        else:
            # Use the latest messages if available
            if self.last_nav2_msg:
                throttle = self._get_value(self.last_nav2_msg, "motor_throttle")
                steering_nav = self._get_value(self.last_nav2_msg, "steering_angle")

            if self.last_lane_msg:
                steering_lane = self._get_value(self.last_lane_msg, "steering_angle")

            # Combine steering (weighted average)
            steering_final = 0.2 * steering_nav + 0.8 * steering_lane

        # Publish final motor command
        cmd = MotorCommands()
        cmd.motor_names = ["steering_angle", "motor_throttle"]
        cmd.values = [steering_final, throttle]

        self.pub_cmd.publish(cmd)
        self.get_logger().debug(f"Publishing: Steering={steering_final:.2f}, Throttle={throttle:.2f}")

    def _get_value(self, msg, name):
        if name in msg.motor_names:
            index = msg.motor_names.index(name)
            return msg.values[index]
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = MotorMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
