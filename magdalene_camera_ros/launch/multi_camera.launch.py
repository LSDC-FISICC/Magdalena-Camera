"""
multi_camera.launch.py
======================
Launch file for starting one magdalene_camera_node per camera defined in
cameras.yaml. Camera names are discovered automatically by scanning the YAML
for top-level sections of the form ``/<name>/<name>``, so adding or removing
a camera only requires editing the config file — no changes to this file needed.

The nodes start a few seconds apart (``stagger_s``) and are respawned when they
exit (``respawn``): the stream watchdog of each node exits when its color rate
stays low, and a fresh process brings a stalled Ethernet camera back.

    ros2 launch magdalene_camera_ros multi_camera.launch.py               # defaults
    ros2 launch magdalene_camera_ros multi_camera.launch.py respawn:=false stagger_s:=0

Author: Julio Fajardo, PhD
"""

import os
import re
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
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


def launch_cameras(context):
    """
    Create one camera node per entry in cameras.yaml, started ``stagger_s`` seconds apart.

    Each node is launched with its own namespace and name matching the camera
    identifier, so topics are published under ``/<name>/color/image_raw`` and
    ``/<name>/infra2/image_raw``. With ``respawn`` the node is started again
    after it exits, each camera with a slightly different delay so that several
    cameras do not restart in lockstep.

    :returns: list of launch actions.
    """
    config = os.path.join(
        get_package_share_directory("magdalene_camera_ros"),
        "config", "cameras.yaml"
    )
    respawn = LaunchConfiguration("respawn").perform(context).lower() in ("true", "1", "yes")
    stagger = float(LaunchConfiguration("stagger_s").perform(context))

    actions = []
    for i, name in enumerate(get_camera_names(config)):
        # Each camera runs as an isolated node in its own namespace
        node = Node(
            package="magdalene_camera_ros",
            executable="magdalene_camera_node",
            name=name,
            namespace=name,
            parameters=[config],
            output="screen",
            respawn=respawn,
            respawn_delay=2.0 + 0.7 * i,
        )
        actions.append(TimerAction(period=stagger * i, actions=[node]) if stagger > 0 and i else node)
    return actions


def generate_launch_description():
    """
    Build a LaunchDescription that spawns one camera node per entry in cameras.yaml.

    :returns: LaunchDescription with the launch arguments and the camera nodes.
    """
    return LaunchDescription([
        DeclareLaunchArgument("respawn", default_value="true",
                              description="start a camera node again when it exits (stream watchdog)"),
        DeclareLaunchArgument("stagger_s", default_value="2.0",
                              description="seconds between the starts of consecutive cameras"),
        OpaqueFunction(function=launch_cameras),
    ])
