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

### Terminal 4 – Hand Control (single or dual arm)
Pick the controller you need:

- Dual-hand follower:
  ```bash
  cd ~/Workspace
  source install/setup.bash
  ros2 run ainex_controller ainex_twohands_control_node 
  ```
- Single-hand practice node:
  ```bash
  cd ~/Workspace
  source install/setup.bash
  ros2 run ainex_controller ainex_hands_control_node 
  ```

Set `sim:=True` when running without hardware so the controller only updates the Pinocchio model instead of commanding the real servos.
