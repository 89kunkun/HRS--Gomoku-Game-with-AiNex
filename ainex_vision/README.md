windows1:
ros2 run ainex_vision undistort_node

windows2:
ros2 run ainex_vision aruco_board_location 

windows3:
ros2 run ainex_vision sym_point

windows4:
ros2 run ainex_vision board_to_base

windows5:
ros2 topic echo /piece_point_base_from_ij

(/piece_point_base_from_ij is the topic that need to be subscribed)

windows6:
ros2 launch ainex_description display.launch.py gui:=false

change the code in line 47 48 of sym_point.py so that you can change the initial point.
