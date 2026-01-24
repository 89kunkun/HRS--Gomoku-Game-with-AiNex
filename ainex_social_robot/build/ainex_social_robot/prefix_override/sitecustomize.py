import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hrs2025/Workspace/src/ainex_social_robot/install/ainex_social_robot'
