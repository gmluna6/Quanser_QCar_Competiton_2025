#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

os.environ["NO_AT_BRIDGE"] = "1"

class RedLightDetectionNode(Node):
    def __init__(self):
        super().__init__('red_light_detection')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/camera/color_image', self.image_callback, 10)

        self.redlight_flag_pub = self.create_publisher(Bool, '/red_light_detected', 10)

        self.stop_in_progress = False
        self.stop_timer = None

        self.get_logger().info("RedLightDetectionNode started. Subscribed to /camera/color_image")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        if frame is None or frame.size == 0:
            self.get_logger().warn("Received empty frame, skipping detection.")
            return

        height, width, _ = frame.shape
        cropped_frame = frame[0:int(height * 0.5), int(width * 0.5):int(width * 0.9)] 

        hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

        # Red color ranges
        lower_red1 = np.array([0, 100, 190])      # Notice the V is now 180 instead of 100
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 190])
        upper_red2 = np.array([180, 255, 255])


        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        # Optional blur to smooth edges
        red_mask = cv2.GaussianBlur(red_mask, (5, 5), 0)

        # White color definition
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        # Contour detection
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_light_detected_this_frame = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 60:  # Filter out small noise
                # Create mask for this contour only
                mask = np.zeros_like(red_mask)
                cv2.drawContours(mask, [cnt], -1, 255, -1)  # Fill the contour

                # Apply mask to cropped HSV frame
                hsv_inside_contour = cv2.bitwise_and(hsv, hsv, mask=mask)

                # White detection inside the contour
                white_mask = cv2.inRange(hsv_inside_contour, lower_white, upper_white)
                white_area = cv2.countNonZero(white_mask)
                contour_area = cv2.contourArea(cnt)

                white_ratio = white_area / (contour_area + 1e-6)  # Avoid division by zero

                self.get_logger().info(f"White ratio in contour: {white_ratio:.2f}")

                if white_ratio < 0.1:  # Accept if less than 10% of the area is white
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cropped_frame, "RED LIGHT DETECTED", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    red_light_detected_this_frame = True
                    break  # Stop after first confirmed detection

        # Detection counter logic
        if red_light_detected_this_frame:
            if not self.stop_in_progress:
                self.stop_in_progress = True
                self.redlight_flag_pub.publish(Bool(data=True))
                self.get_logger().info("RED LIGHT DETECTED - STOPPING CAR")
        else:
            if self.stop_in_progress:
                self.stop_in_progress = False
                self.redlight_flag_pub.publish(Bool(data=False))
                self.get_logger().info("RED LIGHT GONE - RESUMING CAR")


        cv2.imshow("Red Light Detection", cropped_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RedLightDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
