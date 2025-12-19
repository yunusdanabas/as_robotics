from setuptools import find_packages, setup

package_name = 'imu_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/imu_visualization.launch.py',
            'launch/dual_imu.launch.py',
            'launch/dual_imu_spaced.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/imu_view.rviz',
            'rviz/imu_view_position.rviz',
            'rviz/dual_imu_view.rviz',
            'rviz/dual_imu_spaced.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunusdanabas',
    maintainer_email='yunusemredanabas@gmail.com',
    description='IMU visualization tools for tele-imitation project',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_tf_broadcaster = imu_visualization.imu_tf_broadcaster:main',
            'imu_to_euler = imu_visualization.imu_to_euler:main',
            'odometry_tf_broadcaster = imu_visualization.odometry_tf_broadcaster:main',
            'odometry_telemetry = imu_visualization.odometry_telemetry:main',
            'dual_imu_tf_broadcaster = imu_visualization.dual_imu_tf_broadcaster:main',
            'imu_fusion_node = imu_visualization.imu_fusion_node:main',
            'capture_imu_snap = imu_visualization.capture_imu_snap:main',
        ],
    },
)
