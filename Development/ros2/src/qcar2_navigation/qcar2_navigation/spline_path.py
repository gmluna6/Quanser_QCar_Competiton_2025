#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
from scipy.interpolate import splprep, splev

class SplinePathPublisher(Node):
    def __init__(self):
        super().__init__('spline_path_publisher')
        self.path_pub = self.create_publisher(Path, '/spline_path', 10)
        self.timer = self.create_timer(1.0, self.publish_spline_path)

        # Combine all waypoints: pickup + dropoff + return
        all_waypoints = [
            # Pickup
            (-0.33, -1.07),
            (1.5, -1.07),
            (1.92, -0.86),
            (2.20, -0.505),
            (2.20, 0.08),
            (2.20, 3.07),
            (2.20, 3.73),
            (2.20, 4.15),
            (2.02, 4.40),
            (0.86, 4.395),
            (0.125, 4.395),
            # Drop-off
            (-1.2, 4.395),
            (-1.77, 4.09),
            (-1.96, 3.54),
            (-1.96, 1.55),
            (-1.62, 0.98),
            (-0.905, 0.80),
            # Return
            (0.12, 0.94),
            (0.23, 1.57),
            (0.23, 2.17),
            (0.44, 2.50),
            (1.47, 3.10),
            (1.90, 3.17),
            (2.20, 3.73),
            (2.20, 4.15),
            (2.02, 4.40),
            (0.86, 4.395),
            (0.125, 4.395),
            (-1.2, 4.395),
            (-1.77, 4.09),
            (-1.96, 3.54),
            (-1.96, 1.55),
            (-1.92, 0.20),
            (-1.12, -0.90)
        ]

        self.waypoints = np.array(all_waypoints)

    def publish_spline_path(self):
        x = self.waypoints[:, 0]
        y = self.waypoints[:, 1]

        # Fit spline (s=0 forces exact interpolation)
        tck, u = splprep([x, y], s=0)
        u_fine = np.linspace(0, 1, 300)  # More points = smoother path
        x_fine, y_fine = splev(u_fine, tck)

        # Compute yaw from path derivative
        dx = np.gradient(x_fine)
        dy = np.gradient(y_fine)

        # Create Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'qlabs_world'

        for xi, yi, dxi, dyi in zip(x_fine, y_fine, dx, dy):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = xi
            pose.pose.position.y = yi
            pose.pose.position.z = 0.0

            yaw = np.arctan2(dyi, dxi)
            qz = np.sin(yaw / 2.0)
            qw = np.cos(yaw / 2.0)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published spline path with {len(path_msg.poses)} poses.')

def main(args=None):
    rclpy.init(args=args)
    node = SplinePathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
