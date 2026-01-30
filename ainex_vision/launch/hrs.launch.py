from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    undistort_node = Node(
        package="ainex_vision",
        executable="undistort_node",
        name="undistort_node",
        output="screen",
    )

    aruco_board_location = Node(
        package="ainex_vision",
        executable="aruco_board_location",
        name="aruco_board_location",
        output="screen",
    )

    pieces_detection = Node(
        package="ainex_vision",
        executable="pieces_detection",
        name="pieces_detection",
        output="screen",
    )

    arrayoutput = Node(
        package="ainex_vision",
        executable="arrayoutput",
        name="arrayoutput",
        output="screen",
    )

    return LaunchDescription([
        undistort_node,
        aruco_board_location,
        pieces_detection,
        arrayoutput,
    ])



# ros2 run ainex_vision undistort_node
# ros2 run ainex_vision aruco_board_location 
# ros2 run ainex_vision pieces_detection
# ros2 run ainex_vision arrayoutput