from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the undistort, face detection, and social combine nodes."""
    undistort_node = Node(
        package="ainex_vision",
        executable="undistort_node",
        name="undistort_node",
        output="screen",
    )

    face_detection_node = Node(
        package="ainex_vision",
        executable="face_detection_node",
        name="face_detection_node",
        output="screen",
    )

    combine_node = Node(
        package="ainex_social_robot",
        executable="combine_node",
        name="combine_node",
        output="screen",
    )

    return LaunchDescription([
        undistort_node,
        face_detection_node,
        combine_node,
    ])
