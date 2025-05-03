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

class StopSignDetectionNode(Node):
    def __init__(self):
        super().__init__('stop_sign_detection')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/camera/color_image', self.image_callback, 10)

        self.stop_flag_pub = self.create_publisher(Bool, '/stop_sign_detected', 10)

        self.stop_in_progress = False
        self.stop_timer = None

        # NEW: Counter for consecutive detections
        self.detection_counter = 0
        self.detection_threshold = 3  # Number of frames required before triggering stop

        self.get_logger().info("StopSignDetectionNode started. Subscribed to /camera/csi_image")

    def image_callback(self, msg):
        if self.stop_in_progress:
            return  # Skip detection while stopped

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

        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        stop_sign_detected_this_frame = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                peri = cv2.arcLength(cnt, True)
                if peri > 0:
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    vertices = len(approx)
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.putText(cropped_frame, f'{vertices} sides', (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                    if 7 <= vertices <= 9:  # Octagon check
                        cv2.rectangle(cropped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(cropped_frame, 'STOP SIGN DETECTED', (x, y - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        stop_sign_detected_this_frame = True
                        break  # Stop checking other contours once we find one

        # NEW: Detection counter logic
        if stop_sign_detected_this_frame:
            self.detection_counter += 1
            self.get_logger().info(f"STOP sign detection count: {self.detection_counter}")
            if self.detection_counter >= self.detection_threshold:
                self.perform_stop()
                self.detection_counter = 0  # Reset counter after stop is triggered
        else:
            self.detection_counter = 0  # Reset if no detection this frame

        cv2.imshow("Stop Sign Detection", cropped_frame)
        cv2.waitKey(1)

    def perform_stop(self):
        self.stop_in_progress = True
        self.stop_flag_pub.publish(Bool(data=True))
        self.get_logger().info("STOP signal sent after confirmed detections. Waiting 3 seconds...")

        if self.stop_timer is not None:
            self.stop_timer.cancel()

        self.stop_timer = self.create_timer(3.0, self.resume_driving)

    def resume_driving(self):
        self.get_logger().info("Resuming driving after STOP.")
        self.stop_in_progress = False
        self.stop_flag_pub.publish(Bool(data=False))

        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetectionNode()
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
