#!/usr/bin/env python3
"""
IMU Fusion Node

This node subscribes to /imu1_data and /imu2_data topics, fuses their orientations,
and publishes the fused result to /imu_fused. Also broadcasts a TF transform for
the fused orientation.

Fusion Methods:
- SLERP: Spherical linear interpolation with configurable blend factor
- Weighted Average: Normalized weighted sum of quaternion components

Author: IMU Tele-Imitation Project
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def normalize_quaternion(qx, qy, qz, qw):
    """Normalize a quaternion."""
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx/norm, qy/norm, qz/norm, qw/norm


def quaternion_dot(q1, q2):
    """Compute dot product of two quaternions."""
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]


def slerp(q1, q2, t):
    """
    Spherical Linear Interpolation between two quaternions.
    
    Args:
        q1: First quaternion (x, y, z, w) at t=0
        q2: Second quaternion (x, y, z, w) at t=1
        t: Interpolation factor [0.0, 1.0]
    
    Returns:
        Interpolated quaternion (x, y, z, w)
    """
    # Compute dot product
    dot = quaternion_dot(q1, q2)
    
    # If dot is negative, negate one quaternion for shorter path
    q2_adj = q2
    if dot < 0.0:
        q2_adj = (-q2[0], -q2[1], -q2[2], -q2[3])
        dot = -dot
    
    # If quaternions are very close, use linear interpolation
    if dot > 0.9995:
        result = (
            q1[0] + t * (q2_adj[0] - q1[0]),
            q1[1] + t * (q2_adj[1] - q1[1]),
            q1[2] + t * (q2_adj[2] - q1[2]),
            q1[3] + t * (q2_adj[3] - q1[3]),
        )
        return normalize_quaternion(*result)
    
    # Calculate SLERP
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    
    s1 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s2 = sin_theta / sin_theta_0
    
    return (
        q1[0] * s1 + q2_adj[0] * s2,
        q1[1] * s1 + q2_adj[1] * s2,
        q1[2] * s1 + q2_adj[2] * s2,
        q1[3] * s1 + q2_adj[3] * s2,
    )


def weighted_average(q1, q2, w1, w2):
    """
    Weighted average of two quaternions.
    
    Args:
        q1: First quaternion (x, y, z, w)
        q2: Second quaternion (x, y, z, w)
        w1: Weight for first quaternion
        w2: Weight for second quaternion
    
    Returns:
        Averaged quaternion (x, y, z, w)
    """
    # Ensure q2 is in the same hemisphere as q1
    q2_adj = q2
    if quaternion_dot(q1, q2) < 0.0:
        q2_adj = (-q2[0], -q2[1], -q2[2], -q2[3])
    
    # Compute weighted sum
    result = (
        q1[0] * w1 + q2_adj[0] * w2,
        q1[1] * w1 + q2_adj[1] * w2,
        q1[2] * w1 + q2_adj[2] * w2,
        q1[3] * w1 + q2_adj[3] * w2,
    )
    
    return normalize_quaternion(*result)


class QuaternionTracker:
    """Tracks quaternion state for sign continuity and warmup handling."""
    
    def __init__(self, warmup_count=10):
        self.last_quat = None
        self.warmup_remaining = warmup_count
    
    def process(self, qx, qy, qz, qw):
        """Process quaternion with sign continuity."""
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if not math.isfinite(norm) or norm < 1e-6:
            return None
        
        if abs(norm - 1.0) > 1e-3:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        if self.last_quat is not None:
            dot = (qx * self.last_quat[0] + qy * self.last_quat[1] + 
                   qz * self.last_quat[2] + qw * self.last_quat[3])
            if dot < 0.0:
                qx, qy, qz, qw = -qx, -qy, -qz, -qw
        
        self.last_quat = (qx, qy, qz, qw)
        
        if self.warmup_remaining > 0:
            self.warmup_remaining -= 1
            return None
        
        return (qx, qy, qz, qw)


class ImuFusionNode(Node):
    """
    Node that fuses orientation data from two IMU sensors.
    """

    def __init__(self):
        super().__init__('imu_fusion_node')
        
        # Declare parameters
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('fused_frame', 'imu_fused_link')
        self.declare_parameter('imu1_topic', '/imu1_data')
        self.declare_parameter('imu2_topic', '/imu2_data')
        self.declare_parameter('fused_topic', '/imu_fused')
        
        # Fusion parameters
        self.declare_parameter('fusion_method', 'slerp')  # 'slerp' or 'weighted_avg'
        self.declare_parameter('blend_factor', 0.5)  # For SLERP: 0.0=IMU1, 1.0=IMU2
        self.declare_parameter('imu1_weight', 0.5)  # For weighted_avg
        self.declare_parameter('imu2_weight', 0.5)  # For weighted_avg
        
        # Position offset for visualization
        self.declare_parameter('fused_offset_x', 0.0)
        self.declare_parameter('fused_offset_y', 0.0)
        self.declare_parameter('fused_offset_z', 0.0)
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').value
        self.fused_frame = self.get_parameter('fused_frame').value
        imu1_topic = self.get_parameter('imu1_topic').value
        imu2_topic = self.get_parameter('imu2_topic').value
        fused_topic = self.get_parameter('fused_topic').value
        
        self.fusion_method = self.get_parameter('fusion_method').value
        self.blend_factor = self.get_parameter('blend_factor').value
        self.imu1_weight = self.get_parameter('imu1_weight').value
        self.imu2_weight = self.get_parameter('imu2_weight').value
        
        self.fused_offset = (
            self.get_parameter('fused_offset_x').value,
            self.get_parameter('fused_offset_y').value,
            self.get_parameter('fused_offset_z').value,
        )
        
        # Normalize weights
        total_weight = self.imu1_weight + self.imu2_weight
        if total_weight > 0:
            self.imu1_weight /= total_weight
            self.imu2_weight /= total_weight
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create publisher for fused IMU data
        self.fused_publisher = self.create_publisher(Imu, fused_topic, 10)
        
        # Latest data from each IMU
        self.imu1_data = None
        self.imu2_data = None
        
        # Quaternion tracker for fused output
        self.tracker_fused = QuaternionTracker()
        
        # Subscribe to IMU data
        self.sub_imu1 = self.create_subscription(
            Imu,
            imu1_topic,
            self.imu1_callback,
            10
        )
        
        self.sub_imu2 = self.create_subscription(
            Imu,
            imu2_topic,
            self.imu2_callback,
            10
        )
        
        self.get_logger().info('IMU Fusion Node started')
        self.get_logger().info(f'  Method: {self.fusion_method}')
        if self.fusion_method == 'slerp':
            self.get_logger().info(f'  Blend factor: {self.blend_factor}')
        else:
            self.get_logger().info(f'  Weights: IMU1={self.imu1_weight:.2f}, IMU2={self.imu2_weight:.2f}')
        self.get_logger().info(f'  Publishing to: {fused_topic}')
        self.get_logger().info(f'  TF: {self.base_frame} -> {self.fused_frame}')

    def imu1_callback(self, msg):
        """Store IMU1 data and attempt fusion."""
        self.imu1_data = msg
        self._try_fuse()

    def imu2_callback(self, msg):
        """Store IMU2 data and attempt fusion."""
        self.imu2_data = msg
        self._try_fuse()

    def _try_fuse(self):
        """Attempt to fuse IMU data if both are available."""
        if self.imu1_data is None or self.imu2_data is None:
            return
        
        # Extract quaternions
        q1 = (
            self.imu1_data.orientation.x,
            self.imu1_data.orientation.y,
            self.imu1_data.orientation.z,
            self.imu1_data.orientation.w,
        )
        
        q2 = (
            self.imu2_data.orientation.x,
            self.imu2_data.orientation.y,
            self.imu2_data.orientation.z,
            self.imu2_data.orientation.w,
        )
        
        # Validate quaternions
        norm1 = math.sqrt(sum(x*x for x in q1))
        norm2 = math.sqrt(sum(x*x for x in q2))
        
        if norm1 < 1e-6 or norm2 < 1e-6:
            return
        
        # Normalize
        q1 = tuple(x/norm1 for x in q1)
        q2 = tuple(x/norm2 for x in q2)
        
        # Fuse quaternions
        if self.fusion_method == 'slerp':
            q_fused = slerp(q1, q2, self.blend_factor)
        else:
            q_fused = weighted_average(q1, q2, self.imu1_weight, self.imu2_weight)
        
        # Apply sign continuity
        processed = self.tracker_fused.process(*q_fused)
        if processed is None:
            return
        
        qx, qy, qz, qw = processed
        
        # Publish fused IMU message
        fused_msg = Imu()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = self.fused_frame
        
        fused_msg.orientation.x = qx
        fused_msg.orientation.y = qy
        fused_msg.orientation.z = qz
        fused_msg.orientation.w = qw
        
        # Use average of angular velocity and linear acceleration
        fused_msg.angular_velocity.x = (self.imu1_data.angular_velocity.x + 
                                         self.imu2_data.angular_velocity.x) / 2.0
        fused_msg.angular_velocity.y = (self.imu1_data.angular_velocity.y + 
                                         self.imu2_data.angular_velocity.y) / 2.0
        fused_msg.angular_velocity.z = (self.imu1_data.angular_velocity.z + 
                                         self.imu2_data.angular_velocity.z) / 2.0
        
        fused_msg.linear_acceleration.x = (self.imu1_data.linear_acceleration.x + 
                                            self.imu2_data.linear_acceleration.x) / 2.0
        fused_msg.linear_acceleration.y = (self.imu1_data.linear_acceleration.y + 
                                            self.imu2_data.linear_acceleration.y) / 2.0
        fused_msg.linear_acceleration.z = (self.imu1_data.linear_acceleration.z + 
                                            self.imu2_data.linear_acceleration.z) / 2.0
        
        self.fused_publisher.publish(fused_msg)
        
        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = fused_msg.header.stamp
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.fused_frame
        
        t.transform.translation.x = self.fused_offset[0]
        t.transform.translation.y = self.fused_offset[1]
        t.transform.translation.z = self.fused_offset[2]
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImuFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
