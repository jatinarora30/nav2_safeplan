import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jatinarora/nav2_safeplan/install/safeplan_ros2'
