#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import transforms3d.euler
import time

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
from rcl_interfaces.msg import ParameterType

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Publish the static transform qlabs_world → map
        self.publish_static_transform()

        # Phase 1: Pickup waypoints (same as before)
        self.pickup_waypoints = [
            (-0.33, -1.07, 0.0),
            (1.5, -1.07, 0.0),
            (1.92, -0.86, 0.785),
            (2.20, -0.505, 1.571),
            (2.20, 0.08, 1.571),
            (2.30, 3.07, 1.571),
            (2.20, 3.73, 1.571),
            (2.20, 4.15, 1.571),
            (2.02, 4.20, 3.142),
            (0.86, 4.395, 3.142),
            (0.125, 4.395, 3.142)
        ]

        # Phase 2: Drop-off waypoints (same as before)
        self.dropoff_waypoints = [
            (-1.2, 4.395, 3.142),
            (-1.77, 4.09, 3.927),
            (-1.96, 3.54, 4.712),
            (-1.96, 1.55, 4.712),
            (-1.62, 0.98, 5.498),
            (-0.905, 0.80, 0.0)
        ]

        # Phase 3: Return-to-origin waypoints (same as before)
        self.return_waypoints = [
            (0.12, 0.94, 0.785),
            (0.23, 1.57, 1.571),
            (0.23, 2.17, 1.571),
            (0.44, 2.50, 0.785),
            (1.47, 3.10, 0.0),
            (2.00, 3.17, 0.785),
            (2.30, 3.73, 1.571), # RED
            (2.20, 4.15, 1.571),
            (2.02, 4.20, 3.142),
            (0.86, 4.395, 3.142),
            (0.125, 4.395, 3.142), # RED
            (-1.2, 4.395, 3.142), # BLUE
            (-1.77, 4.09, 3.927),
            (-1.96, 3.54, 4.712),
            (-1.96, 1.55, 4.712), # BLUE
            (-1.92, 0.20, 4.712),
            (-1.12, -0.90, 0.785)
        ]

        self.current_path = self.pickup_waypoints
        self.current_index = 0
        self.trip_stage = "pickup"  # stages: pickup → dropoff → return

    def publish_static_transform(self):
        self.static_broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'qlabs_world'
        t.child_frame_id = 'map'
        t.transform.translation.x = -1.120
        t.transform.translation.y = -0.906
        t.transform.translation.z = 0.0

        q = transforms3d.euler.euler2quat(0.0, 0.0, -0.60)  # Roll, Pitch, Yaw

        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info("Static transform qlabs_world → map published.")

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
        q = transforms3d.euler.euler2quat(0.0, 0.0, yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'qlabs_world'  # ✅ Changed here!
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
            self.set_led_color(1)
            time.sleep(3)
            self.current_path = self.dropoff_waypoints
        elif self.trip_stage == "dropoff":
            self.trip_stage = "return"
            self.set_led_color(2)
            time.sleep(3)
            self.current_path = self.return_waypoints
        else:
            self.get_logger().info("All trips completed.")
            self.set_led_color(0)
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
