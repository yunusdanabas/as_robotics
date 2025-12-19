"""
Launch file for Dual IMU visualization

Starts:
- Dual IMU TF broadcaster (broadcasts imu1_link, imu2_link)
- IMU fusion node (publishes /imu_fused, broadcasts imu_fused_link)
- Optional: Euler angle publishers for each IMU
- RViz2 with dual IMU visualization configuration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('imu_visualization')
    
    # Path to RViz config file
    rviz_dual_config = os.path.join(pkg_dir, 'rviz', 'dual_imu_view.rviz')
    
    # Declare launch arguments
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='world',
        description='Base frame for TF tree'
    )
    
    imu1_frame_arg = DeclareLaunchArgument(
        'imu1_frame',
        default_value='imu1_link',
        description='IMU1 (BNO055) frame name'
    )
    
    imu2_frame_arg = DeclareLaunchArgument(
        'imu2_frame',
        default_value='imu2_link',
        description='IMU2 (MPU6050) frame name'
    )
    
    fused_frame_arg = DeclareLaunchArgument(
        'fused_frame',
        default_value='imu_fused_link',
        description='Fused IMU frame name'
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
    
    # Fusion parameters
    fusion_method_arg = DeclareLaunchArgument(
        'fusion_method',
        default_value='slerp',
        description='Fusion method: slerp or weighted_avg'
    )
    
    blend_factor_arg = DeclareLaunchArgument(
        'blend_factor',
        default_value='0.5',
        description='SLERP blend factor: 0.0=IMU1 only, 1.0=IMU2 only, 0.5=equal'
    )
    
    imu1_weight_arg = DeclareLaunchArgument(
        'imu1_weight',
        default_value='0.5',
        description='IMU1 weight for weighted average fusion'
    )
    
    imu2_weight_arg = DeclareLaunchArgument(
        'imu2_weight',
        default_value='0.5',
        description='IMU2 weight for weighted average fusion'
    )
    
    # Euler output parameters
    publish_euler_arg = DeclareLaunchArgument(
        'publish_euler',
        default_value='false',
        description='Publish roll/pitch/yaw for each IMU'
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
    
    # Topic names
    imu1_topic_arg = DeclareLaunchArgument(
        'imu1_topic',
        default_value='/imu1_data',
        description='IMU1 (BNO055) data topic'
    )
    
    imu2_topic_arg = DeclareLaunchArgument(
        'imu2_topic',
        default_value='/imu2_data',
        description='IMU2 (MPU6050) data topic'
    )
    
    fused_topic_arg = DeclareLaunchArgument(
        'fused_topic',
        default_value='/imu_fused',
        description='Fused IMU data topic'
    )
    
    # Visual offset parameters for TF frames
    imu1_offset_y_arg = DeclareLaunchArgument(
        'imu1_offset_y',
        default_value='-0.3',
        description='Y offset for IMU1 frame visualization'
    )
    
    imu2_offset_y_arg = DeclareLaunchArgument(
        'imu2_offset_y',
        default_value='0.3',
        description='Y offset for IMU2 frame visualization'
    )
    
    # Dual IMU TF Broadcaster node
    dual_imu_tf_broadcaster_node = Node(
        package='imu_visualization',
        executable='dual_imu_tf_broadcaster',
        name='dual_imu_tf_broadcaster',
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'imu1_frame': LaunchConfiguration('imu1_frame'),
            'imu2_frame': LaunchConfiguration('imu2_frame'),
            'imu1_topic': LaunchConfiguration('imu1_topic'),
            'imu2_topic': LaunchConfiguration('imu2_topic'),
            'imu1_offset_y': LaunchConfiguration('imu1_offset_y'),
            'imu2_offset_y': LaunchConfiguration('imu2_offset_y'),
        }],
        output='screen',
    )
    
    # IMU Fusion node
    imu_fusion_node = Node(
        package='imu_visualization',
        executable='imu_fusion_node',
        name='imu_fusion_node',
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'fused_frame': LaunchConfiguration('fused_frame'),
            'imu1_topic': LaunchConfiguration('imu1_topic'),
            'imu2_topic': LaunchConfiguration('imu2_topic'),
            'fused_topic': LaunchConfiguration('fused_topic'),
            'fusion_method': LaunchConfiguration('fusion_method'),
            'blend_factor': LaunchConfiguration('blend_factor'),
            'imu1_weight': LaunchConfiguration('imu1_weight'),
            'imu2_weight': LaunchConfiguration('imu2_weight'),
        }],
        output='screen',
    )
    
    # Euler publisher for IMU1
    euler_imu1_node = Node(
        package='imu_visualization',
        executable='imu_to_euler',
        name='imu1_to_euler',
        parameters=[
            {
                'input_topic': LaunchConfiguration('imu1_topic'),
                'output_topic': '/imu1/euler',
                'use_degrees': LaunchConfiguration('euler_use_degrees'),
                'print_values': LaunchConfiguration('euler_print_values'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('publish_euler')),
        output='screen',
    )
    
    # Euler publisher for IMU2
    euler_imu2_node = Node(
        package='imu_visualization',
        executable='imu_to_euler',
        name='imu2_to_euler',
        parameters=[
            {
                'input_topic': LaunchConfiguration('imu2_topic'),
                'output_topic': '/imu2/euler',
                'use_degrees': LaunchConfiguration('euler_use_degrees'),
                'print_values': LaunchConfiguration('euler_print_values'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('publish_euler')),
        output='screen',
    )
    
    # Euler publisher for fused IMU
    euler_fused_node = Node(
        package='imu_visualization',
        executable='imu_to_euler',
        name='fused_to_euler',
        parameters=[
            {
                'input_topic': LaunchConfiguration('fused_topic'),
                'output_topic': '/imu_fused/euler',
                'use_degrees': LaunchConfiguration('euler_use_degrees'),
                'print_values': LaunchConfiguration('euler_print_values'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('publish_euler')),
        output='screen',
    )
    
    def launch_rviz(context, *_args, **_kwargs):
        use_rviz = LaunchConfiguration('use_rviz').perform(context).lower()
        if use_rviz not in ('true', '1', 'yes'):
            return []

        override_path = LaunchConfiguration('rviz_config').perform(context)
        config_path = rviz_dual_config if not override_path else override_path

        node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_path],
            output='screen'
        )
        return [node]

    rviz_action = OpaqueFunction(function=launch_rviz)
    
    return LaunchDescription([
        # Arguments
        base_frame_arg,
        imu1_frame_arg,
        imu2_frame_arg,
        fused_frame_arg,
        use_rviz_arg,
        rviz_config_override_arg,
        fusion_method_arg,
        blend_factor_arg,
        imu1_weight_arg,
        imu2_weight_arg,
        publish_euler_arg,
        euler_use_degrees_arg,
        euler_print_arg,
        imu1_topic_arg,
        imu2_topic_arg,
        fused_topic_arg,
        imu1_offset_y_arg,
        imu2_offset_y_arg,
        # Nodes
        dual_imu_tf_broadcaster_node,
        imu_fusion_node,
        euler_imu1_node,
        euler_imu2_node,
        euler_fused_node,
        rviz_action,
    ])
