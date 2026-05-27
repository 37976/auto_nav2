#!/usr/bin/env python3
"""odom_to_map_relay.py -- 将 /odom (odom 坐标系) 变换到 map 坐标系后转发."""

import math
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomToMapRelay(Node):
    def __init__(self):
        super().__init__("odom_to_map_relay")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("output_topic", "/odom_in_map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        odom_topic = str(self.get_parameter("odom_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 20)
        self._odom_pub = self.create_publisher(Odometry, output_topic, 20)

        self.get_logger().info(
            f"odom→map 转发: {odom_topic} → {output_topic}")

    def _odom_cb(self, msg: Odometry):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._odom_frame, rclpy.time.Time())
        except TransformException:
            return

        t = tf.transform.translation
        yaw_tf = _yaw(tf.transform.rotation)
        yaw_odom = _yaw(msg.pose.pose.orientation)
        yaw_map = math.atan2(math.sin(yaw_tf + yaw_odom),
                             math.cos(yaw_tf + yaw_odom))

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        map_x = t.x + (odom_x * math.cos(yaw_tf) - odom_y * math.sin(yaw_tf))
        map_y = t.y + (odom_x * math.sin(yaw_tf) + odom_y * math.cos(yaw_tf))

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = map_x
        out.pose.pose.position.y = map_y
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.z = math.sin(yaw_map * 0.5)
        out.pose.pose.orientation.w = math.cos(yaw_map * 0.5)
        out.twist = msg.twist
        self._odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
