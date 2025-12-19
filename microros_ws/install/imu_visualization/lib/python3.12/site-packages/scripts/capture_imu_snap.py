#!/usr/bin/env python3
"""
IMU Snap Event Capture Tool

Captures IMU quaternion data and detects sudden yaw snap/teleportation events.
Logs raw topic data around snap events for analysis.

Usage:
    python3 capture_imu_snap.py [--topic /imu_data] [--threshold 0.3] [--output snap_capture.csv]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import math
import csv
import argparse
from datetime import datetime
from collections import deque


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_dot(q1, q2):
    """Compute dot product of two quaternions."""
    return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w


def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in radians."""
    dot = abs(quaternion_dot(q1, q2))
    dot = max(-1.0, min(1.0, dot))  # Clamp to [-1, 1]
    return 2.0 * math.acos(dot)


class ImuSnapCapture(Node):
    """Node that captures IMU data and detects snap events."""

    def __init__(self, topic, threshold, output_file):
        super().__init__('imu_snap_capture')
        
        self.topic = topic
        self.threshold = threshold  # Angular change threshold in radians
        self.output_file = output_file
        
        # Previous quaternion for comparison
        self.prev_quat = None
        self.prev_yaw = None
        self.sample_count = 0
        
        # Buffer to store recent samples (for context around snap)
        self.buffer_size = 50  # Store last 50 samples (~1 second at 50Hz)
        self.sample_buffer = deque(maxlen=self.buffer_size)
        
        # CSV file for logging
        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_sec', 'timestamp_nsec', 'sample_count',
            'q_w', 'q_x', 'q_y', 'q_z',
            'q_norm', 'dot_prev', 'angular_diff_rad', 'angular_diff_deg',
            'roll_rad', 'pitch_rad', 'yaw_rad',
            'yaw_deg', 'yaw_diff_rad', 'yaw_diff_deg',
            'gyro_x', 'gyro_y', 'gyro_z',
            'accel_x', 'accel_y', 'accel_z',
            'snap_detected', 'snap_magnitude_rad'
        ])
        
        # QoS profile for micro-ROS compatibility (best_effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            self.topic,
            self.imu_callback,
            qos_profile
        )
        
        self.get_logger().info(f'IMU Snap Capture started')
        self.get_logger().info(f'Topic: {self.topic}')
        self.get_logger().info(f'Threshold: {self.threshold} rad ({math.degrees(self.threshold):.2f} deg)')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info('Waiting for IMU data...')

    def imu_callback(self, msg):
        """Callback for IMU data. Detects snaps and logs data."""
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Normalize quaternion
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-6:
            self.get_logger().warn('Received invalid quaternion (zero norm)')
            return
        
        if abs(norm - 1.0) > 1e-3:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
        
        # Apply sign continuity
        if self.prev_quat is not None:
            dot = qx * self.prev_quat.x + qy * self.prev_quat.y + qz * self.prev_quat.z + qw * self.prev_quat.w
            if dot < 0.0:
                qx = -qx
                qy = -qy
                qz = -qz
                qw = -qw
        
        # Compute metrics
        self.sample_count += 1
        angular_diff = 0.0
        dot_prev = 0.0
        yaw_diff = 0.0
        snap_detected = False
        snap_magnitude = 0.0
        
        if self.prev_quat is not None:
            # Angular distance
            angular_diff = quaternion_angular_distance(msg.orientation, self.prev_quat)
            dot_prev = quaternion_dot(msg.orientation, self.prev_quat)
            
            # Yaw difference (with unwrap)
            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
            if self.prev_yaw is not None:
                yaw_diff = yaw - self.prev_yaw
                # Unwrap yaw difference (handle ±π boundary)
                if yaw_diff > math.pi:
                    yaw_diff -= 2 * math.pi
                elif yaw_diff < -math.pi:
                    yaw_diff += 2 * math.pi
                
                # Check for snap
                if angular_diff > self.threshold:
                    snap_detected = True
                    snap_magnitude = angular_diff
                    self.get_logger().warn(
                        f'SNAP DETECTED! Sample #{self.sample_count}: '
                        f'angular_diff={angular_diff:.4f} rad ({math.degrees(angular_diff):.2f} deg), '
                        f'yaw_diff={yaw_diff:.4f} rad ({math.degrees(yaw_diff):.2f} deg)'
                    )
            else:
                roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        else:
            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        
        # Store sample in buffer
        sample_data = {
            'timestamp_sec': msg.header.stamp.sec,
            'timestamp_nsec': msg.header.stamp.nanosec,
            'sample_count': self.sample_count,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'norm': norm,
            'dot_prev': dot_prev,
            'angular_diff': angular_diff,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'yaw_diff': yaw_diff,
            'gyro_x': msg.angular_velocity.x,
            'gyro_y': msg.angular_velocity.y,
            'gyro_z': msg.angular_velocity.z,
            'accel_x': msg.linear_acceleration.x,
            'accel_y': msg.linear_acceleration.y,
            'accel_z': msg.linear_acceleration.z,
            'snap_detected': snap_detected,
            'snap_magnitude': snap_magnitude
        }
        self.sample_buffer.append(sample_data)
        
        # Write to CSV
        self.csv_writer.writerow([
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            self.sample_count,
            qw, qx, qy, qz,
            norm,
            dot_prev,
            angular_diff,
            math.degrees(angular_diff),
            roll, pitch, yaw,
            math.degrees(yaw),
            yaw_diff,
            math.degrees(yaw_diff),
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            1 if snap_detected else 0,
            snap_magnitude
        ])
        self.csv_file.flush()  # Ensure data is written immediately
        
        # Update previous values
        self.prev_quat = msg.orientation
        self.prev_yaw = yaw
        
        # Log snap event with context
        if snap_detected:
            self.get_logger().info(f'Snap event logged to {self.output_file}')
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='Capture IMU data and detect snap events')
    parser.add_argument('--topic', type=str, default='/imu_data',
                       help='IMU topic to subscribe (default: /imu_data)')
    parser.add_argument('--threshold', type=float, default=0.3,
                       help='Angular change threshold in radians (default: 0.3 rad = ~17 deg)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output CSV file (default: snap_capture_TIMESTAMP.csv)')
    
    args = parser.parse_args()
    
    # Generate output filename if not provided
    if args.output is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output = f'snap_capture_{timestamp}.csv'
    
    rclpy.init(args=None)
    
    node = ImuSnapCapture(args.topic, args.threshold, args.output)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

