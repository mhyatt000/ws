"""Bringup launch file that composes the camera example."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("xnodes_camera"), "/launch/camera.launch.py"]
        )
    )

    return LaunchDescription([camera_launch])
