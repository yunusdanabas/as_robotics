import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yunusdanabas/as_robotics/microros_ws/install/imu_visualization'
