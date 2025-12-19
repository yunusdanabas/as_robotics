#!/usr/bin/env python3
"""
IMU TF Broadcaster Node

This node subscribes to the /imu_data topic and broadcasts TF transforms
to visualize the IMU orientation in RViz2.

Author: IMU Tele-Imitation Project
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class ImuTfBroadcaster(Node):
    """
    Node that broadcasts TF transforms based on IMU orientation data.
    """

    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        
        # Declare parameters
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('imu_topic', '/imu_data')
        self.declare_parameter('translation_x', 0.0)
        self.declare_parameter('translation_y', 0.0)
        self.declare_parameter('translation_z', 0.0)
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.translation_x = self.get_parameter('translation_x').value
        self.translation_y = self.get_parameter('translation_y').value
        self.translation_z = self.get_parameter('translation_z').value
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS profile for micro-ROS compatibility (best_effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to IMU data (use configurable topic)
        self.subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile
        )
        
        self.get_logger().info(f'IMU TF Broadcaster started')
        self.get_logger().info(f'Broadcasting: {self.base_frame} -> {self.imu_frame}')
        self.get_logger().info(f'Listening to: {self.imu_topic}')

        self._invalid_quat_warned = False
        self._last_quat = None
        self._warmup = 10

    def imu_callback(self, msg):
        """
        Callback for IMU data. Broadcasts TF transform.
        """
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if (not math.isfinite(norm)) or norm < 1e-6:
            if not self._invalid_quat_warned:
                self.get_logger().warn(
                    f'Received invalid orientation (norm={norm}). Skipping TF publish until valid data arrives.'
                )
                self._invalid_quat_warned = True
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
        self._invalid_quat_warned = False

        # Create transform message
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.imu_frame
        
        # Set translation (with configurable offset)
        t.transform.translation.x = self.translation_x
        t.transform.translation.y = self.translation_y
        t.transform.translation.z = self.translation_z
        
        # Set rotation from IMU quaternion
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImuTfBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

