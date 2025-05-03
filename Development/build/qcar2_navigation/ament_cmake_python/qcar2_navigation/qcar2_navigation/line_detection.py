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

class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/csi_image',
            self.image_callback,
            10
        )

        self.motor_pub = self.create_publisher(
            MotorCommands,
            'qcar2_motor_speed_cmd/lane_following',
            10
        )

        self.get_logger().info("LaneFollowerNode with Bird’s-Eye + Curve Fitting + PD Controller started.")

        # PD controller parameters
        self.kp = 3.0    # Proportional gain
        self.kd = 1.0    # Derivative gain

        # PD internal states
        self.last_center_offset = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]

        # Bird's-eye view warp
        src = np.float32([
            [int(0.05 * width), height],
            [int(0.45 * width), int(0.55 * height)],
            [int(0.55 * width), int(0.55 * height)],
            [int(0.95 * width), height]
        ])
        dst = np.float32([
            [0, height],
            [0, 0],
            [width, 0],
            [width, height]
        ])
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(frame, M, (width, height))

        # Preprocessing
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold to find bright lane markings
        _, binary = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY)

        # Histogram to find lane base
        histogram = np.sum(binary[binary.shape[0]//2:, :], axis=0)
        midpoint = np.int32(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Sliding windows
        nwindows = 9
        window_height = np.int32(binary.shape[0] / nwindows)
        nonzero = binary.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        margin = 50
        minpix = 50

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary.shape[0] - (window + 1) * window_height
            win_y_high = binary.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Fit polynomials
        if len(leftx) > 0 and len(rightx) > 0:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            ploty = np.linspace(0, binary.shape[0] - 1, binary.shape[0])
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

            # Lane center
            lane_center = (left_fitx[-1] + right_fitx[-1]) / 2
            car_center = binary.shape[1] / 2
            center_offset = (car_center - lane_center) / (binary.shape[1] / 2)

            # Time step
            current_time = self.get_clock().now().nanoseconds / 1e9
            dt = current_time - self.last_time if self.last_time else 1e-6

            # PD Controller
            p_term = self.kp * center_offset
            d_term = self.kd * (center_offset - self.last_center_offset) / dt
            steering_angle = p_term + d_term

            # Update memory
            self.last_center_offset = center_offset
            self.last_time = current_time

            # Cap steering (optional small limit)
            steering_angle = np.clip(steering_angle, -1.0, 1.0)

            forward_speed = 1.0  # Constant speed for now

            # Publish MotorCommand
            cmd = MotorCommands()
            cmd.motor_names = ['steering_angle', 'motor_throttle']
            cmd.values = [steering_angle, forward_speed]
            self.motor_pub.publish(cmd)

            self.get_logger().info(f"Steering: {steering_angle:.3f}, Offset: {center_offset:.3f}")

        else:
            self.get_logger().warn("Lanes not detected!")

        # Optional debug view
        cv2.imshow("Bird's Eye View", binary)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
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
