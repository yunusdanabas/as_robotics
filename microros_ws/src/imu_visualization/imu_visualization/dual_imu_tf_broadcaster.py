#!/usr/bin/env python3
"""
Dual IMU TF Broadcaster Node

This node subscribes to /imu1_data and /imu2_data topics and broadcasts TF transforms
to visualize both IMU orientations in RViz2.

Features:
- Quaternion normalization
- Sign continuity handling
- Warm-up period handling
- Configurable frame offsets for spatial separation

Author: IMU Tele-Imitation Project
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class QuaternionTracker:
    """Tracks quaternion state for sign continuity and warmup handling."""
    
    def __init__(self, warmup_count=10):
        self.last_quat = None
        self.warmup_remaining = warmup_count
        self.invalid_warned = False
    
    def process(self, qx, qy, qz, qw):
        """
        Process incoming quaternion with normalization, sign continuity, and warmup.
        
        Returns:
            tuple: (qx, qy, qz, qw, is_valid) where is_valid indicates if past warmup
        """
        # Check for valid quaternion
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if not math.isfinite(norm) or norm < 1e-6:
            return qx, qy, qz, qw, False
        
        # Normalize if needed
        if abs(norm - 1.0) > 1e-3:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
        
        # Apply sign continuity
        if self.last_quat is not None:
            dot = (qx * self.last_quat[0] + qy * self.last_quat[1] + 
                   qz * self.last_quat[2] + qw * self.last_quat[3])
            if dot < 0.0:
                qx, qy, qz, qw = -qx, -qy, -qz, -qw
        
        self.last_quat = (qx, qy, qz, qw)
        
        # Handle warmup period
        if self.warmup_remaining > 0:
            self.warmup_remaining -= 1
            return qx, qy, qz, qw, False
        
        self.invalid_warned = False
        return qx, qy, qz, qw, True


class DualImuTfBroadcaster(Node):
    """
    Node that broadcasts TF transforms for two IMU sensors.
    """

    def __init__(self):
        super().__init__('dual_imu_tf_broadcaster')
        
        # Declare parameters
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('imu1_frame', 'imu1_link')
        self.declare_parameter('imu2_frame', 'imu2_link')
        self.declare_parameter('imu1_topic', '/imu1_data')
        self.declare_parameter('imu2_topic', '/imu2_data')
        
        # Position offsets for visualization (to separate the frames visually)
        self.declare_parameter('imu1_offset_x', 0.0)
        self.declare_parameter('imu1_offset_y', -0.3)
        self.declare_parameter('imu1_offset_z', 0.0)
        self.declare_parameter('imu2_offset_x', 0.0)
        self.declare_parameter('imu2_offset_y', 0.3)
        self.declare_parameter('imu2_offset_z', 0.0)
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').value
        self.imu1_frame = self.get_parameter('imu1_frame').value
        self.imu2_frame = self.get_parameter('imu2_frame').value
        imu1_topic = self.get_parameter('imu1_topic').value
        imu2_topic = self.get_parameter('imu2_topic').value
        
        self.imu1_offset = (
            self.get_parameter('imu1_offset_x').value,
            self.get_parameter('imu1_offset_y').value,
            self.get_parameter('imu1_offset_z').value,
        )
        self.imu2_offset = (
            self.get_parameter('imu2_offset_x').value,
            self.get_parameter('imu2_offset_y').value,
            self.get_parameter('imu2_offset_z').value,
        )
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Quaternion trackers for each IMU
        self.tracker1 = QuaternionTracker()
        self.tracker2 = QuaternionTracker()
        
        # Subscribe to IMU1 data
        self.sub_imu1 = self.create_subscription(
            Imu,
            imu1_topic,
            self.imu1_callback,
            10
        )
        
        # Subscribe to IMU2 data
        self.sub_imu2 = self.create_subscription(
            Imu,
            imu2_topic,
            self.imu2_callback,
            10
        )
        
        self.get_logger().info('Dual IMU TF Broadcaster started')
        self.get_logger().info(f'  IMU1: {imu1_topic} -> {self.base_frame} -> {self.imu1_frame}')
        self.get_logger().info(f'  IMU2: {imu2_topic} -> {self.base_frame} -> {self.imu2_frame}')

    def _broadcast_transform(self, frame_id, offset, qx, qy, qz, qw):
        """Broadcast a TF transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = frame_id
        
        t.transform.translation.x = offset[0]
        t.transform.translation.y = offset[1]
        t.transform.translation.z = offset[2]
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)

    def imu1_callback(self, msg):
        """Callback for IMU1 (BNO055) data."""
        qx, qy, qz, qw, valid = self.tracker1.process(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        if valid:
            self._broadcast_transform(self.imu1_frame, self.imu1_offset, qx, qy, qz, qw)

    def imu2_callback(self, msg):
        """Callback for IMU2 (MPU6050) data."""
        qx, qy, qz, qw, valid = self.tracker2.process(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        if valid:
            self._broadcast_transform(self.imu2_frame, self.imu2_offset, qx, qy, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    
    node = DualImuTfBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
