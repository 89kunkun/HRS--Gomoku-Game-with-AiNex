# Gomoku Visualization 

## 运行
```bash
cd ~/gomoku_visualization
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run gomoku_visualization gomoku_visualizer_node
```

## topic
- `/gomoku/move` `Int16MultiArray`：`[row, col, player]`，player `1=black`，`2=white`。越界或落子冲突会在界面提示。
- `/gomoku/warning` `String`：自定义警告文字，显示在状态栏。
- `/gomoku/reset` `Empty`：清空棋盘与历史。

棋盘大小16

## 快速测试指令
```bash
# 下两步棋（黑再白）
ros2 topic pub --once /gomoku/move std_msgs/msg/Int16MultiArray "{data: [7, 7, 1]}"
ros2 topic pub --once /gomoku/move std_msgs/msg/Int16MultiArray "{data: [7, 8, 2]}"

# 发送警告
ros2 topic pub --once /gomoku/warning std_msgs/msg/String "data: 'occupied cell'"

# 重置棋盘
ros2 topic pub --once /gomoku/reset std_msgs/msg/Empty "{}"
```
