# AiNex Run Guide
### Groupe Member
Shuowen Li, Yuan Feng, Aokun Zhang, Haishan Xia, Chang Song

This guide explains how to bring up the full vision/control pipeline—image undistortion, ArUco detection, and hand controllers—across multiple terminals.

## 1. Environment & Build
1. Open a terminal and move to the workspace root (`~/Workspace/src` lives under `~/Workspace`):
   ```bash
   cd ~/Workspace
   ```
2. Install/update dependencies and build (run once initially or whenever code changes):
   ```bash
   colcon build
   ```
3. After building, remember to source the workspace **in every terminal** before launching any node:
   ```bash
   source install/setup.bash
   ```

## Exercise 1 - Walking with teleoperation
```bash
cd ~/Workspace
source install/setup.bash
ros2 run ainex_motion walking_keyboard_node
```
Keyboard Control:
  w/s : forward/backward
  a/d : strafe left/right
  q/e : rotate left/right
  space : stop
  +/- : speed up/down
  CTRL-C : quit

## Exercise 2 - Walk Towards a Target (Aruco)
## 2. Launch Order (4 Terminals)
> Each of the following commands must run in its own terminal. Launch them one by one.

### Terminal 1 – Camera Undistortion
```bash
cd ~/Workspace
source install/setup.bash
ros2 run ainex_vision undistort_node
```
Subscribes to `/camera_image/compressed`, publishes `/camera/image_undistorted`, `/camera/image_undistorted/compressed`, and `/camera/camera_info`.

### Terminal 2 – ArUco Detection
```bash
cd ~/Workspace
source install/setup.bash
ros2 run ainex_vision aruco_detector
```
Make sure it receives the undistorted image stream plus `CameraInfo`. It publishes marker poses on `/aruco_marker_base_link` and broadcasts TF.

### Terminal 3 – RViz Display (URDF Visualization)
```bash
cd ~/Workspace
source install/setup.bash
ros2 launch ainex_description display.launch.py gui:=false
```
Brings up the URDF visualization so you can monitor joint states (set `gui:=true` if you need interactive joint sliders).

### Terminal 4 – Hand Control (Aruco Follow)
```bash
cd ~/Workspace
source install/setup.bash
ros2 run ainex_motion aruco_follower_node
```
This node implements an ArUco-based following behavior using a simple FSM (SEARCH → FOLLOW → STOP).

1. SEARCH: the robot keeps its base stationary and scans the environment by smoothly moving the head.
2. FOLLOW: once the marker is detected, head motion stops and the robot walks toward the marker using velocity control.
3. STOP: the robot stops when the desired distance is reached.