import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/michal/Desktop/ros2_project/ros2_ws/install/camera_driver'
