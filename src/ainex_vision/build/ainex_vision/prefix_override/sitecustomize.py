import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hrs2025/Workspace/src/ainex_vision/install/ainex_vision'
