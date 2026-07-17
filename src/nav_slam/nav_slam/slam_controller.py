#!/usr/bin/env python3
"""
slam_controller: Bridge between slam_toolbox and the DashGo navigation stack.

- Subscribes to /slam_map (from slam_toolbox) with transient_local QoS
- Relays /slam_map to /combined_grid for the navigation stack
- Provides ~/save_map service to save the current SLAM map as PGM+YAML
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

from nav_slam.map_saver import save_occupancy_grid


class SlamController(Node):
    """Bridge between slam_toolbox and DashGo navigation stack."""

    def __init__(self):
        super().__init__("slam_controller")

        # --- Parameters ---
        self.declare_parameter("slam_map_topic", "/slam_map")
        self.declare_parameter("combined_grid_topic", "/combined_grid")
        self.declare_parameter("map_save_dir", "")
        self.declare_parameter("relay_rate_hz", 2.0)
        self.declare_parameter("map_frame_id", "map")

        self.slam_map_topic = str(self.get_parameter("slam_map_topic").value)
        self.combined_grid_topic = str(self.get_parameter("combined_grid_topic").value)
        self.map_save_dir = str(self.get_parameter("map_save_dir").value)
        self.relay_rate_hz = float(self.get_parameter("relay_rate_hz").value)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)

        # Default save directory: nav_slam/map/
        if not self.map_save_dir:
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.map_save_dir = os.path.join(pkg_dir, "map")

        # --- State ---
        self.latest_slam_map: OccupancyGrid | None = None
        self.save_filename = "dashgo_slam_map"

        # --- Subscriptions ---
        # transient_local: get the latest map even if published before we subscribed
        slam_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, self.slam_map_topic, self.slam_map_callback, slam_qos
        )

        # --- Publications ---
        self.combined_grid_pub = self.create_publisher(
            OccupancyGrid, self.combined_grid_topic, 10
        )

        # --- Relay timer ---
        self.relay_timer = self.create_timer(
            1.0 / max(self.relay_rate_hz, 0.5), self.relay_callback
        )

        # --- Services ---
        self.save_srv = self.create_service(Trigger, "~/save_map", self.save_map_callback)

        self.get_logger().info("slam_controller started (sim mode)")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def slam_map_callback(self, msg: OccupancyGrid):
        """Cache the latest SLAM map."""
        self.latest_slam_map = msg

    def relay_callback(self):
        """Publish the latest SLAM map to /combined_grid."""
        if self.latest_slam_map is None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.latest_slam_map.header.frame_id or self.map_frame_id
        grid.info = self.latest_slam_map.info
        grid.data = self.latest_slam_map.data
        self.combined_grid_pub.publish(grid)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def save_map_callback(self, request, response):
        """Save current SLAM map to PGM+YAML files."""
        if self.latest_slam_map is None:
            response.success = False
            response.message = "No SLAM map data available yet"
            return response

        try:
            pgm_path, yaml_path = save_occupancy_grid(
                self.latest_slam_map, self.save_filename, self.map_save_dir
            )
            response.success = True
            response.message = f"Map saved: {pgm_path}"
            self.get_logger().info(f"Map saved to {pgm_path}")
        except Exception as e:
            response.success = False
            response.message = f"Save failed: {e}"
            self.get_logger().error(f"Map save failed: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SlamController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
