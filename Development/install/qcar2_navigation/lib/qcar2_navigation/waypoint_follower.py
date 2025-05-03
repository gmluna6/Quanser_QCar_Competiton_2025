#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import transforms3d.euler


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 🔧 Define waypoints here (x, y, yaw in radians)
        self.waypoints = [
            (1.0, 0.4, 0.7853),
            (2.3, 1.5, 0.7853),
            (2.3, 2.7, 2.3561),
            (0.2, 5.7, 2.3561),
            (-0.1, 6.1, 2.6179),
            (-1.1, 5.8, -2.3561),
            (-2.0, 5.2, -2.3561)
        ]
        self.current_index = 0

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed.')
            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[self.current_index]
        q = transforms3d.euler.euler2quat(0.0, 0.0, yaw)  # (w, x, y, z)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = q[1]
        goal_msg.pose.pose.orientation.y = q[2]
        goal_msg.pose.pose.orientation.z = q[3]
        goal_msg.pose.pose.orientation.w = q[0]

        self.get_logger().info(f'Sending goal {self.current_index + 1}: ({x:.2f}, {y:.2f}, {yaw:.2f} rad)')
        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal {self.current_index + 1} completed with result: {result}')
        self.current_index += 1
        self.send_next_goal()  # 🔁 Send the next goal


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    node.send_next_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
