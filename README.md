# HRS -- Gomoku Game with AiNex Project

This document provides a quick-start overview for running the perception, game logic, visualization, and manipulation components of the HRS Gomoku project. Each section lists the minimal ROS 2 commands to launch the corresponding subsystem.

## Perception: Board and Move Localization

### Run the perception part
```bash
   ros2 launch ainex_vision hrs.launch.py 
```


## Game Logic and Real-time Board Visualization 
### Game Logic
```bash
   $ ros2 run gomoku_manager gomoku_manager_node 
```

### Real-time Board Visualization 
```bash
   $ ros2 run gomoku_ui gomoku_ui_payload_visualizer 
```


## Manipulation: Drawing Dots with Cartesian Contro
### Visualiye the robot in RViz
```bash
   ros2 launch ainex_description display.launch.py gui:=false
```

### Run Arm Controller
```bash
   ros2 run ainex_controller ainex_arm_controller_topic 
```

### TF transfor the point form optical_camera_link into base_link
```bash
   ros2 run ainex_vision sym_point
   ros2 run ainex_vision board_to_base
```
