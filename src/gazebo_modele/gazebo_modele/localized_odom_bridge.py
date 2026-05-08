#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_to_matrix(qx, qy, qz, qw):
    sqx = qx * qx
    sqy = qy * qy
    sqz = qz * qz
    return [
        [1.0 - 2.0 * (sqy + sqz), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
        [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (sqx + sqz), 2.0 * (qy * qz - qw * qx)],
        [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (sqx + sqy)],
    ]


def mat_vec_mul(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class LocalizedOdomBridge(Node):
    def __init__(self):
        super().__init__("localized_odom_bridge")
        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter("output_odom_topic", "/localized_odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("lookup_timeout_sec", 0.05)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("log_pose", True)
        self.declare_parameter("log_period_sec", 0.5)

        self.input_odom_topic = self.get_parameter("input_odom_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.lookup_timeout = Duration(seconds=float(self.get_parameter("lookup_timeout_sec").value))
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.log_pose = bool(self.get_parameter("log_pose").value)
        self.log_period_sec = max(0.1, float(self.get_parameter("log_period_sec").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.odom_sub = self.create_subscription(Odometry, self.input_odom_topic, self.odom_cb, 20)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self._warned = False
        self._last_logged_x: Optional[float] = None
        self._last_logged_y: Optional[float] = None
        self._last_logged_t: Optional[float] = None
        self._last_motion_state: Optional[str] = None

    def odom_cb(self, msg: Odometry):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=self.lookup_timeout,
            )
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.odom_frame,
                    rclpy.time.Time(),
                    timeout=self.lookup_timeout,
                )
            except TransformException as exc:
                if not self._warned:
                    self.get_logger().warn(f"Waiting for {self.map_frame}->{self.odom_frame} TF: {exc}")
                    self._warned = True
                return

        self._warned = False
        tq = tf.transform.rotation
        tt = tf.transform.translation
        tf_rot = quat_to_matrix(tq.x, tq.y, tq.z, tq.w)
        pose = msg.pose.pose.position
        rotated = mat_vec_mul(tf_rot, [pose.x, pose.y, pose.z])

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = rotated[0] + tt.x
        out.pose.pose.position.y = rotated[1] + tt.y
        out.pose.pose.position.z = rotated[2] + tt.z

        oq = msg.pose.pose.orientation
        rx, ry, rz, rw = quat_multiply((tq.x, tq.y, tq.z, tq.w), (oq.x, oq.y, oq.z, oq.w))
        out.pose.pose.orientation.x = rx
        out.pose.pose.orientation.y = ry
        out.pose.pose.orientation.z = rz
        out.pose.pose.orientation.w = rw
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        self.odom_pub.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        if self.log_pose:
            self._log_pose_status(out)

    def _log_pose_status(self, msg: Odometry):
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

        if self._last_logged_x is None or self._last_logged_y is None:
            step_xy = 0.0
        else:
            step_xy = math.hypot(x - self._last_logged_x, y - self._last_logged_y)
        moving = step_xy > 0.005
        motion_state = "MOVING" if moving else "STABLE"

        if self._last_logged_t is not None and stamp_sec - self._last_logged_t < self.log_period_sec:
            return
        if motion_state == "STABLE" and self._last_motion_state == "STABLE":
            return

        self._last_logged_x = x
        self._last_logged_y = y
        self._last_logged_t = stamp_sec
        self._last_motion_state = motion_state

        self.get_logger().info(
            "Localized pose %s | x=%.2f y=%.2f yaw=%.1f deg"
            % (
                motion_state,
                x,
                y,
                math.degrees(yaw),
            )
        )

    @staticmethod
    def _yaw_from_quaternion(quat) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizedOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
