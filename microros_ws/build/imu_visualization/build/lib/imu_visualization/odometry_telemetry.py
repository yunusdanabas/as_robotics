#!/usr/bin/env python3
"""Derive telemetry (Euler angles, velocity, altitude) from nav_msgs/Odometry."""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32


def quaternion_to_euler(x: float, y: float, z: float, w: float,
                        last_quat: Optional[Tuple[float, float, float, float]]):
    """Convert quaternion to roll/pitch/yaw while enforcing sign continuity."""
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


class OdometryTelemetry(Node):
    """Publish derived telemetry from the odometry stream."""

    def __init__(self):
        super().__init__('odometry_telemetry')

        self.declare_parameter('input_topic', '/imu_odometry')
        self.declare_parameter('euler_topic', '/imu/pose/euler')
        self.declare_parameter('velocity_topic', '/imu/pose/velocity')
        self.declare_parameter('altitude_topic', '/imu/pose/altitude')
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('publish_velocity', True)
        self.declare_parameter('publish_altitude', True)
        self.declare_parameter('publish_path', True)
        self.declare_parameter('path_topic', '/imu/pose/path')
        self.declare_parameter('path_length', 500)
        self.declare_parameter('warmup_samples', 10)

        self.input_topic = self.get_parameter('input_topic').value
        self.euler_topic = self.get_parameter('euler_topic').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.altitude_topic = self.get_parameter('altitude_topic').value
        self.use_degrees = bool(self.get_parameter('use_degrees').value)
        self.publish_velocity = bool(self.get_parameter('publish_velocity').value)
        self.publish_altitude = bool(self.get_parameter('publish_altitude').value)
        self.publish_path = bool(self.get_parameter('publish_path').value)
        self.path_topic = self.get_parameter('path_topic').value
        self.max_path_length = int(self.get_parameter('path_length').value)
        self.warmup = int(self.get_parameter('warmup_samples').value)

        self._last_quat: Optional[Tuple[float, float, float, float]] = None

        self.euler_pub = self.create_publisher(Vector3Stamped, self.euler_topic, 10)
        self.velocity_pub = None
        self.altitude_pub = None
        self.path_pub = None
        self.path_msg = Path()

        if self.publish_velocity:
            self.velocity_pub = self.create_publisher(TwistStamped, self.velocity_topic, 10)

        if self.publish_altitude:
            self.altitude_pub = self.create_publisher(Float32, self.altitude_topic, 10)

        if self.publish_path:
            self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(
            f'Odometry telemetry node listening to {self.input_topic}. Euler -> {self.euler_topic}'
        )

    def odom_callback(self, msg: Odometry):
        if self.warmup > 0:
            self.warmup -= 1
            return

        quat, self._last_quat = quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            self._last_quat,
        )

        if quat is None:
            return

        roll, pitch, yaw = self._quat_to_euler(quat)

        euler_msg = Vector3Stamped()
        euler_msg.header = msg.header
        euler_msg.vector.x = roll
        euler_msg.vector.y = pitch
        euler_msg.vector.z = yaw
        self.euler_pub.publish(euler_msg)

        if self.velocity_pub is not None:
            twist_msg = TwistStamped()
            twist_msg.header = msg.header
            twist_msg.twist = msg.twist.twist
            self.velocity_pub.publish(twist_msg)

        if self.altitude_pub is not None:
            altitude_msg = Float32()
            altitude_msg.data = float(msg.pose.pose.position.z)
            self.altitude_pub.publish(altitude_msg)

        if self.path_pub is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.path_msg.header = msg.header
            self.path_msg.poses.append(pose_stamped)
            if len(self.path_msg.poses) > self.max_path_length:
                self.path_msg.poses = self.path_msg.poses[-self.max_path_length:]
            self.path_pub.publish(self.path_msg)

    def _quat_to_euler(self, quat: Tuple[float, float, float, float]):
        qx, qy, qz, qw = quat

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.use_degrees:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = OdometryTelemetry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


