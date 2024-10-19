import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yasmin-nunes/ros2-path-planning/install/yolov8_ros'
