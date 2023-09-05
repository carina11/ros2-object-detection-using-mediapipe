import os

# ROS
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generates LaunchDescription for object detection."""
    param_file = os.path.join(
        get_package_share_directory("object_detection"), "params", "object_detection.yaml"
    )

    object_detection = Node(
        name="object_detection_node",
        package="object_detection",
        executable="object_detection_node.py",
        emulate_tty=True,
        output={"both": {"screen", "log", "own_log"}},
        parameters=[
            param_file,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )


    nodes_list = [object_detection]

    return LaunchDescription(nodes_list)
