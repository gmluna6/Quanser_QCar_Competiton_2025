#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__('simple_goal_sender_node')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self):
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal_msg.pose.pose.position.x = -1.0
        goal_msg.pose.pose.position.y = 3.492

        # Set orientation for 90° yaw (facing up)
        goal_msg.pose.pose.orientation.z = 2.35619
        goal_msg.pose.pose.orientation.w = 2.35619

        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation completed with result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
