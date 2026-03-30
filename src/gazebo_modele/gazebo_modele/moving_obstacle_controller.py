#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetEntityState, SetModelState


class MovingObstacleController(Node):
    def __init__(self):
        super().__init__('moving_obstacle_controller')

        self.declare_parameter('enabled', True)
        self.declare_parameter('entity_name', 'dynamic_box_1')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', -6.0)
        self.declare_parameter('base_z', 0.5)
        self.declare_parameter('amp_x', 3.0)
        self.declare_parameter('amp_y', 0.0)
        self.declare_parameter('period', 10.0)
        self.declare_parameter('yaw', 0.0)

        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self.entity_name = self.get_parameter('entity_name').get_parameter_value().string_value
        self.reference_frame = self.get_parameter(
            'reference_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.base_x = self.get_parameter('base_x').get_parameter_value().double_value
        self.base_y = self.get_parameter('base_y').get_parameter_value().double_value
        self.base_z = self.get_parameter('base_z').get_parameter_value().double_value
        self.amp_x = self.get_parameter('amp_x').get_parameter_value().double_value
        self.amp_y = self.get_parameter('amp_y').get_parameter_value().double_value
        self.period = max(self.get_parameter('period').get_parameter_value().double_value, 0.1)
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.entity_clients = [
            ('/gazebo/set_entity_state', self.create_client(SetEntityState, '/gazebo/set_entity_state')),
            ('/set_entity_state', self.create_client(SetEntityState, '/set_entity_state')),
        ]
        self.model_clients = [
            ('/gazebo/set_model_state', self.create_client(SetModelState, '/gazebo/set_model_state')),
            ('/set_model_state', self.create_client(SetModelState, '/set_model_state')),
        ]
        self.active_client = None
        self.active_service_name = ''
        self.active_mode = ''
        self.pending_future = None
        self.last_wait_log_ns = 0
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self.timer_callback)

        self.get_logger().info(
            f'MovingObstacleController started for {self.entity_name}, enabled={self.enabled}')

    def try_connect_client(self):
        if self.active_client is not None and self.active_client.service_is_ready():
            return True

        for service_name, client in self.entity_clients:
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.0):
                self.active_client = client
                self.active_service_name = service_name
                self.active_mode = 'entity'
                self.get_logger().info(
                    f'Connected to Gazebo service {service_name} using SetEntityState')
                return True

        for service_name, client in self.model_clients:
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.0):
                self.active_client = client
                self.active_service_name = service_name
                self.active_mode = 'model'
                self.get_logger().info(
                    f'Connected to Gazebo service {service_name} using SetModelState')
                return True

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_wait_log_ns > 5_000_000_000:
            self.get_logger().warn(
                'No Gazebo state service is ready yet. Waiting for one of: '
                '/gazebo/set_entity_state, /set_entity_state, '
                '/gazebo/set_model_state, /set_model_state')
            self.last_wait_log_ns = now_ns
        return False

    def on_request_done(self, future):
        self.pending_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to move {self.entity_name}: {exc}')
            return

        if hasattr(response, 'success') and not response.success:
            status = getattr(response, 'status_message', '')
            self.get_logger().warn(
                f'Gazebo rejected motion for {self.entity_name}: {status}')

    def timer_callback(self):
        if not self.enabled:
            return
        if not self.try_connect_client():
            return
        if self.pending_future is not None and not self.pending_future.done():
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        phase = 2.0 * math.pi * elapsed / self.period

        x = self.base_x + self.amp_x * math.sin(phase)
        y = self.base_y + self.amp_y * math.sin(phase)

        state = EntityState()
        state.name = self.entity_name
        state.reference_frame = self.reference_frame
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = self.base_z
        state.pose.orientation.z = math.sin(self.yaw * 0.5)
        state.pose.orientation.w = math.cos(self.yaw * 0.5)

        if self.active_mode == 'entity':
            request = SetEntityState.Request()
            request.state = state
        else:
            model_state = ModelState()
            model_state.model_name = self.entity_name
            model_state.reference_frame = self.reference_frame
            model_state.pose = state.pose
            request = SetModelState.Request()
            request.model_state = model_state

        self.pending_future = self.active_client.call_async(request)
        self.pending_future.add_done_callback(self.on_request_done)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
