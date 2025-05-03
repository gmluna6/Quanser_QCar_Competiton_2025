#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import transforms3d.euler
import time

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, SetParametersAtomically
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Phase 1: Pickup waypoints
        self.pickup_waypoints = [
            (1.0, 0.4, 0.7853), #1
            (2.5, 1.5, 0.7853), #2
            (2.75, 2.0, 1.5708), #3
            (2.35, 2.7, 2.3561), #4
            (0.2, 5.7, 2.3561), #5
            (-0.1, 6.1, 2.6179), #6
            (-1.3, 5.5, -2.3561), #7
            (-2.0, 5.0, -2.3561) #8
        ]


        # Phase 2: Drop-off waypoints
        self.dropoff_waypoints = [
            (-3.0, 4.4, -2.3561), #9
            (-3.40, 3.8, -1.5708), #10
            (-3.3, 3.3, -0.7853), #11
            (-2.15, 1.6, -0.7853), #12
            (-1.4, 1.2, 0.0), #13
            (-0.6, 1.7, 0.7853) #14
        ]


        # Phase 3: Return-to-origin waypoints
        self.return_waypoints = [
            (0.0, 2.3, 1.5708), #15
            (-0.2, 2.7, 2.3561), #16
            (-0.7, 3.3, 1.5708), #17
            (-0.7, 4.0, 1.5708), #18
            (-0.3, 4.7, 0.7853), #19
            (0.3, 5.0, 0.7853), #20
            (0.45, 5.4, 1.5708), #21
            (0.2, 5.7, 2.3561), #5
            (-0.1, 6.2, 2.6179), #6
            (-1.3, 5.5, -2.3561), #7
            (-2.0, 5.0, -2.3561), #8
            (-3.0, 4.4, -2.3561), #9
            (-3.40, 3.8, -1.5708), #10
            (-3.3, 3.3, -0.7853), #11
            (-2.15, 1.6, -0.7853), # 12
            (-1.7, 1.0, -0.7853), #22
            (-1.0, 0.2, 0.0), #23
            (0.0, 0.0, 0.0) #0
        ]

        self.current_path = self.pickup_waypoints
        self.current_index = 0
        self.trip_stage = "pickup"  # stages: pickup → dropoff → return

    def set_led_color(self, color_id):
        self.led_client = self.create_client(SetParameters, '/qcar2_hardware/set_parameters')
        if not self.led_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('LED service not available!')
            return

        param = ParameterMsg(
            name='led_color_id',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=color_id)
        )
        request = SetParameters.Request(parameters=[param])

        self.led_future = self.led_client.call_async(request)
        self.led_future.add_done_callback(self.led_response_callback)

    def led_response_callback(self, future):
        if future.result() is not None:
            self.get_logger().info("LED color successfully set.")
        else:
            self.get_logger().error("Failed to set LED color.")



    def send_next_goal(self):
        if self.current_index >= len(self.current_path):
            self.get_logger().info(f"{self.trip_stage.capitalize()} trip completed.")
            self.transition_to_next_trip()
            return

        x, y, yaw = self.current_path[self.current_index]
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

        self.get_logger().info(f'Sending goal {self.current_index + 1}/{len(self.current_path)} for {self.trip_stage}...')
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
        self.get_logger().info(f'Goal {self.current_index + 1} result: {result}')
        self.current_index += 1
        self.send_next_goal()

    def transition_to_next_trip(self):
        if self.trip_stage == "pickup":
            self.trip_stage = "dropoff"
            self.set_led_color(1)  # Green for pickup
            time.sleep(3)
            self.current_path = self.dropoff_waypoints
        elif self.trip_stage == "dropoff":
            self.trip_stage = "return"
            self.set_led_color(2)  # Blue for drop-off
            time.sleep(3)
            self.current_path = self.return_waypoints
        else:
            self.get_logger().info("All trips completed.")
            self.set_led_color(0)  # Back to red
            rclpy.shutdown()
            return
      
        self.current_index = 0
        self.get_logger().info(f"Starting {self.trip_stage} trip.")
        self.send_next_goal()


    def start_delayed_next_leg(self):
        self.get_logger().info(f"Starting {self.trip_stage} trip.")
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    node.send_next_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()