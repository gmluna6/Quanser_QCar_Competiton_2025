#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

from qcar2_interfaces.msg import MotorCommands

os.environ["NO_AT_BRIDGE"] = "1"

class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/csi_image', self.image_callback, 10)
        self.motor_pub = self.create_publisher(MotorCommands, 'qcar2_motor_speed_cmd/lane_following', 10)

        self.get_logger().info("LineDetectionNode with PID started. Subscribed to /camera/csi_image")

        # PID parameters
        self.kp = 5.5   # Proportional gain
        self.ki = 0.0   # Integral gain
        self.kd = 25.0   # Derivative gain

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]

        # Grayscale + blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Region of interest mask
        mask = np.zeros_like(gray)
        roi_corners = np.array([[
            (int(0.05 * width), height),
            (int(0.45 * width), int(0.55 * height)),
            (int(0.55 * width), int(0.55 * height)),
            (int(0.95 * width), height)
        ]], dtype=np.int32)
        cv2.fillPoly(mask, roi_corners, 255)
        masked = cv2.bitwise_and(blurred, mask)

        # Edge detection
        edges = cv2.Canny(masked, 50, 150)

        # Hough transform for line detection
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=40, maxLineGap=20)

        # Lane logic
        line_image = np.copy(frame)
        steering_angle = 0.0
        forward_speed = 1.0  # base throttle

        left_x, right_x = [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)

                if abs(slope) < 0.5:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    continue

                if slope < 0:
                    left_x.extend([x1, x2])
                elif slope > 0:
                    right_x.extend([x1, x2])

                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if left_x and right_x:
            lane_center = (min(right_x) + max(left_x)) / 2
            frame_center = width / 2
            error = (lane_center - frame_center) / frame_center

            # Time step
            current_time = self.get_clock().now().nanoseconds / 1e9
            dt = current_time - self.last_time if self.last_time else 1e-6

            # PID calculations
            self.integral += error * dt
            derivative = (error - self.last_error) / dt

            steering_angle = -(self.kp * error + self.ki * self.integral + self.kd * derivative)

            # Save for next cycle
            self.last_error = error
            self.last_time = current_time

        # Publish motor command
        cmd = MotorCommands()
        cmd.motor_names = ['steering_angle', 'motor_throttle']
        cmd.values = [steering_angle, forward_speed]
        self.get_logger().info(f"Steering (PID): {steering_angle:.2f}")
        self.motor_pub.publish(cmd)

        # Debug window
        cv2.imshow("Lines", line_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
