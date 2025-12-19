#!/usr/bin/env python3
"""TF broadcaster for IMU odometry."""

from typing import Optional, Tuple

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def normalize_quaternion(x: float, y: float, z: float, w: float,
                         last_quat: Optional[Tuple[float, float, float, float]]):
    """Normalize quaternion and enforce sign continuity."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if (not math.isfinite(norm)) or norm < 1e-6:
        return None, last_quat

    if abs(norm - 1.0) > 1e-3:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    if last_quat is not None:
        dot = x * last_quat[0] + y * last_quat[1] + z * last_quat[2] + w * last_quat[3]
        if dot < 0.0:
            x = -x
            y = -y
            z = -z
            w = -w

    return (x, y, z, w), (x, y, z, w)


class OdometryTfBroadcaster(Node):
    """Broadcast TF using nav_msgs/Odometry from the ESP32."""

    def __init__(self):
        super().__init__('odometry_tf_broadcaster')

        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('odom_topic', '/imu_odometry')
        self.declare_parameter('warmup_samples', 10)

        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.warmup_samples = int(self.get_parameter('warmup_samples').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self._warmup = self.warmup_samples
        self._last_quat: Optional[Tuple[float, float, float, float]] = None

        self.get_logger().info(
            f'Odometry TF Broadcaster active: {self.base_frame} -> {self.imu_frame} '
            f'from topic {self.odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        quat, self._last_quat = normalize_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            self._last_quat,
        )

        if quat is None:
            self.get_logger().warn_once('Received invalid odometry quaternion; skipping TF publication.')
            return

        if self._warmup > 0:
            self._warmup -= 1
            return

        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.imu_frame
        transform.transform.translation = msg.pose.pose.position
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


