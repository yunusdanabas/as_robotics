#!/usr/bin/env python3
"""IMU Euler angle publisher.

Subscribes to `/imu_data` (sensor_msgs/msg/Imu), converts the quaternion to
roll, pitch, yaw (degrees by default) and publishes the result on
`/imu/euler` (geometry_msgs/msg/Vector3Stamped).  Optionally prints the
values to stdout for quick inspection.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuEulerNode(Node):
    """Node converting IMU quaternion data to roll/pitch/yaw angles."""

    def __init__(self):
        super().__init__('imu_to_euler')

        self.declare_parameter('input_topic', '/imu_data')
        self.declare_parameter('output_topic', '/imu/euler')
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('print_values', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.use_degrees = self.get_parameter('use_degrees').get_parameter_value().bool_value
        self.print_values = self.get_parameter('print_values').get_parameter_value().bool_value

        self._last_quat = None
        self._warmup = 10

        # QoS profile for micro-ROS compatibility (best_effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(Vector3Stamped, output_topic, 10)
        self.subscription = self.create_subscription(Imu, input_topic, self.imu_callback, qos_profile)

        unit = 'deg' if self.use_degrees else 'rad'
        self.get_logger().info(
            f"Publishing roll/pitch/yaw on '{output_topic}' ({unit}). "
            f"Source topic: '{input_topic}'."
        )

    def imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if (not math.isfinite(norm)) or norm < 1e-6:
            return

        if abs(norm - 1.0) > 1e-3:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        if self._last_quat is not None:
            dot = qx * self._last_quat[0] + qy * self._last_quat[1] + qz * self._last_quat[2] + qw * self._last_quat[3]
            if dot < 0.0:
                qx = -qx
                qy = -qy
                qz = -qz
                qw = -qw

        if self._warmup > 0:
            self._warmup -= 1
            self._last_quat = (qx, qy, qz, qw)
            return

        self._last_quat = (qx, qy, qz, qw)

        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        if self.use_degrees:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

        out_msg = Vector3Stamped()
        out_msg.header = msg.header
        out_msg.vector.x = roll
        out_msg.vector.y = pitch
        out_msg.vector.z = yaw
        self.publisher.publish(out_msg)

        if self.print_values:
            unit = 'deg' if self.use_degrees else 'rad'
            self.get_logger().info(
                f'roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f} ({unit})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuEulerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

