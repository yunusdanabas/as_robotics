"""
Launch file for IMU visualization

Starts:
- IMU TF broadcaster
- RViz2 with IMU visualization configuration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('imu_visualization')
    
    # Path to RViz config file
    rviz_orientation_config = os.path.join(pkg_dir, 'rviz', 'imu_view.rviz')
    rviz_position_config = os.path.join(pkg_dir, 'rviz', 'imu_view_position.rviz')
    
    # Declare launch arguments
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='world',
        description='Base frame for TF tree'
    )
    
    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame',
        default_value='imu_link',
        description='IMU frame name'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2'
    )

    rviz_config_override_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Optional RViz config path override'
    )

    use_position_mode_arg = DeclareLaunchArgument(
        'use_position_mode',
        default_value='false',
        description='Enable odometry-based visualization (position mode)'
    )

    publish_euler_arg = DeclareLaunchArgument(
        'publish_euler',
        default_value='true',
        description='Publish roll/pitch/yaw as Vector3Stamped'
    )

    euler_use_degrees_arg = DeclareLaunchArgument(
        'euler_use_degrees',
        default_value='true',
        description='Euler output in degrees (set false for radians)'
    )

    euler_print_arg = DeclareLaunchArgument(
        'euler_print_values',
        default_value='false',
        description='Print roll/pitch/yaw to console'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu_data',
        description='IMU topic to subscribe'
    )

    euler_topic_arg = DeclareLaunchArgument(
        'euler_topic',
        default_value='/imu/euler',
        description='Euler output topic'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/imu_odometry',
        description='Odometry topic produced by the ESP32 firmware'
    )

    pose_euler_topic_arg = DeclareLaunchArgument(
        'pose_euler_topic',
        default_value='/imu/pose/euler',
        description='Euler output topic when position mode is active'
    )

    pose_velocity_topic_arg = DeclareLaunchArgument(
        'pose_velocity_topic',
        default_value='/imu/pose/velocity',
        description='Velocity topic when position mode is active'
    )

    pose_altitude_topic_arg = DeclareLaunchArgument(
        'pose_altitude_topic',
        default_value='/imu/pose/altitude',
        description='Altitude topic when position mode is active'
    )

    position_publish_velocity_arg = DeclareLaunchArgument(
        'position_publish_velocity',
        default_value='true',
        description='Publish TwistStamped velocity in position mode'
    )

    position_publish_altitude_arg = DeclareLaunchArgument(
        'position_publish_altitude',
        default_value='true',
        description='Publish altitude Float32 in position mode'
    )

    pose_path_topic_arg = DeclareLaunchArgument(
        'pose_path_topic',
        default_value='/imu/pose/path',
        description='Path topic when position mode is active'
    )

    position_path_length_arg = DeclareLaunchArgument(
        'position_path_length',
        default_value='500',
        description='Number of pose samples to retain in the path'
    )

    position_publish_path_arg = DeclareLaunchArgument(
        'position_publish_path',
        default_value='true',
        description='Publish nav_msgs/Path in position mode'
    )
    
    # IMU TF Broadcaster node
    imu_tf_broadcaster_node = Node(
        package='imu_visualization',
        executable='imu_tf_broadcaster',
        name='imu_tf_broadcaster',
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'imu_frame': LaunchConfiguration('imu_frame'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_position_mode'))
    )
    
    def launch_rviz(context, *_args, **_kwargs):
        use_rviz = LaunchConfiguration('use_rviz').perform(context).lower()
        if use_rviz not in ('true', '1', 'yes'):
            return []

        override_path = LaunchConfiguration('rviz_config').perform(context)
        config_path = rviz_orientation_config
        if override_path:
            config_path = override_path
        else:
            use_position = LaunchConfiguration('use_position_mode').perform(context).lower()
            if use_position in ('true', '1', 'yes'):
                config_path = rviz_position_config

        node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_path],
            output='screen'
        )
        return [node]

    rviz_action = OpaqueFunction(function=launch_rviz)

    euler_node = Node(
        package='imu_visualization',
        executable='imu_to_euler',
        name='imu_to_euler',
        parameters=[
            {
                'input_topic': LaunchConfiguration('imu_topic'),
                'output_topic': LaunchConfiguration('euler_topic'),
                'use_degrees': LaunchConfiguration('euler_use_degrees'),
                'print_values': LaunchConfiguration('euler_print_values'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('publish_euler')),
        output='screen',
    )

    odom_tf_node = Node(
        package='imu_visualization',
        executable='odometry_tf_broadcaster',
        name='odometry_tf_broadcaster',
        parameters=[
            {
                'base_frame': LaunchConfiguration('base_frame'),
                'imu_frame': LaunchConfiguration('imu_frame'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('use_position_mode')),
        output='screen'
    )

    odom_telemetry_node = Node(
        package='imu_visualization',
        executable='odometry_telemetry',
        name='odometry_telemetry',
        parameters=[
            {
                'input_topic': LaunchConfiguration('odom_topic'),
                'euler_topic': LaunchConfiguration('pose_euler_topic'),
                'velocity_topic': LaunchConfiguration('pose_velocity_topic'),
                'altitude_topic': LaunchConfiguration('pose_altitude_topic'),
                'use_degrees': LaunchConfiguration('euler_use_degrees'),
                'publish_velocity': LaunchConfiguration('position_publish_velocity'),
                'publish_altitude': LaunchConfiguration('position_publish_altitude'),
                'publish_path': LaunchConfiguration('position_publish_path'),
                'path_topic': LaunchConfiguration('pose_path_topic'),
                'path_length': LaunchConfiguration('position_path_length'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('use_position_mode')),
        output='screen'
    )
    
    return LaunchDescription([
        base_frame_arg,
        imu_frame_arg,
        use_rviz_arg,
        rviz_config_override_arg,
        use_position_mode_arg,
        publish_euler_arg,
        euler_use_degrees_arg,
        euler_print_arg,
        imu_topic_arg,
        euler_topic_arg,
        odom_topic_arg,
        pose_euler_topic_arg,
        pose_velocity_topic_arg,
        pose_altitude_topic_arg,
        position_publish_velocity_arg,
        position_publish_altitude_arg,
        pose_path_topic_arg,
        position_path_length_arg,
        position_publish_path_arg,
        imu_tf_broadcaster_node,
        euler_node,
        odom_tf_node,
        odom_telemetry_node,
        rviz_action,
    ])

