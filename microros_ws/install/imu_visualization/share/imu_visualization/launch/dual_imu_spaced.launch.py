"""
Launch file for Dual IMU visualization with 8cm spacing

Layout:
- IMU1 (BNO055): -0.04m (4cm left)
- IMU2 (MPU6050): +0.04m (4cm right)  
- Fused: 0.0m (center)
Total spacing: 8cm

Starts:
- Three TF broadcasters (one for each IMU)
- RViz2 with dual IMU visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('imu_visualization')
    
    # Path to RViz config file
    rviz_config = os.path.join(pkg_dir, 'rviz', 'dual_imu_spaced.rviz')
    
    # Declare launch arguments
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='world',
        description='Base frame for TF tree'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2'
    )
    
    # IMU1 (BNO055) TF Broadcaster - 4cm to the left
    imu1_tf_node = Node(
        package='imu_visualization',
        executable='imu_tf_broadcaster',
        name='imu1_tf_broadcaster',
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'imu_frame': 'imu1_link',
            'imu_topic': '/imu1_data',
            'translation_x': 0.0,
            'translation_y': -0.04,  # 4cm left
            'translation_z': 0.0,
        }],
        output='screen',
    )
    
    # IMU2 (MPU6050) TF Broadcaster - 4cm to the right
    imu2_tf_node = Node(
        package='imu_visualization',
        executable='imu_tf_broadcaster',
        name='imu2_tf_broadcaster',
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'imu_frame': 'imu2_link',
            'imu_topic': '/imu2_data',
            'translation_x': 0.0,
            'translation_y': 0.04,  # 4cm right
            'translation_z': 0.0,
        }],
        output='screen',
    )
    
    # Fused IMU TF Broadcaster - center
    fused_tf_node = Node(
        package='imu_visualization',
        executable='imu_tf_broadcaster',
        name='fused_tf_broadcaster',
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'imu_frame': 'imu_fused_link',
            'imu_topic': '/imu_fused',
            'translation_x': 0.0,
            'translation_y': 0.0,  # Center
            'translation_z': 0.0,
        }],
        output='screen',
    )
    
    def launch_rviz(context, *_args, **_kwargs):
        use_rviz = LaunchConfiguration('use_rviz').perform(context).lower()
        if use_rviz not in ('true', '1', 'yes'):
            return []

        node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
        return [node]

    rviz_action = OpaqueFunction(function=launch_rviz)
    
    return LaunchDescription([
        base_frame_arg,
        use_rviz_arg,
        imu1_tf_node,
        imu2_tf_node,
        fused_tf_node,
        rviz_action,
    ])


