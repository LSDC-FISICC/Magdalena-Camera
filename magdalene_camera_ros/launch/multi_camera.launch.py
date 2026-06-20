"""
multi_camera.launch.py
======================
Launch file for starting one magdalene_camera_node per camera defined in
cameras.yaml. Camera names are discovered automatically by scanning the YAML
for top-level sections of the form ``/<name>/<name>``, so adding or removing
a camera only requires editing the config file — no changes to this file needed.

Author: Julio Fajardo, PhD
"""

import os
import re
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def get_camera_names(config_path: str) -> list:
    """
    Extract camera names from a ROS 2 parameter YAML file.

    Looks for top-level keys of the form ``/<name>/<name>`` (e.g.
    ``/camera_55/camera_55``) and returns the unique name component of each
    matching section, sorted alphabetically.

    :param config_path: Absolute path to the cameras.yaml config file.
    :returns: Sorted list of camera name strings.
    """
    with open(config_path) as f:
        data = yaml.safe_load(f)
    cameras = []
    for key in data:
        # Match sections where the namespace and node name are identical
        m = re.fullmatch(r'/(\w+)/(\w+)', key)
        if m and m.group(1) == m.group(2):
            cameras.append(m.group(1))
    return sorted(cameras)


def generate_launch_description():
    """
    Build a LaunchDescription that spawns one camera node per entry in cameras.yaml.

    Each node is launched with its own namespace and name matching the camera
    identifier, so topics are published under ``/<name>/color/image_raw`` and
    ``/<name>/infra2/image_raw``.

    :returns: LaunchDescription containing all camera nodes.
    """
    config = os.path.join(
        get_package_share_directory("magdalene_camera_ros"),
        "config", "cameras.yaml"
    )

    nodes = []
    for name in get_camera_names(config):
        # Each camera runs as an isolated node in its own namespace
        nodes.append(
            Node(
                package="magdalene_camera_ros",
                executable="magdalene_camera_node",
                name=name,
                namespace=name,
                parameters=[config],
                output="screen",
            )
        )
    return LaunchDescription(nodes)
