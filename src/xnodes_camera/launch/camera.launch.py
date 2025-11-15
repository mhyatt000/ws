"""Launch description for the xnodes_camera example node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="xnodes_camera",
                executable="camera_node",
                name="xnodes_camera",
                output="screen",
            )
        ]
    )
