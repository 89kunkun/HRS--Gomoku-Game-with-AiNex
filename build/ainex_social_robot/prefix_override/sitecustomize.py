import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hrs2025/git_website/HRS--Gomoku-Game-with-AiNex/install/ainex_social_robot'
