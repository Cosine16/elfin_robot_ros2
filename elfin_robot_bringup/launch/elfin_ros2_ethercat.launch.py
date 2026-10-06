#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# DEEP MERGE: override keys win; nested dicts are merged recursively.
def deep_merge(base, override):
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = deep_merge(result[k], v)
        else:
            result[k] = v
    return result

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #

def generate_launch_description():
    pkg_bringup = "elfin_robot_bringup"
    base = load_yaml(pkg_bringup, os.path.join("config", "elfin_drivers.yaml")) or {}

    # Per-machine local override (git-ignored, e.g. elfin_drivers.local.yaml).
    # Create it ONLY on machines whose config differs from the committed baseline.
    local = load_yaml(pkg_bringup, os.path.join("config", "elfin_drivers.local.yaml"))
    if local is not None:
        base = deep_merge(base, local)

    elfin_ethercat_node = Node(
        name="elfin_ethercat_driver_node",
        package = "elfin_ethercat_driver",
        executable = "elfin_ethercat_driver",
        output = "screen",
        parameters = [base]
    )

    return LaunchDescription(
        [
            elfin_ethercat_node
        ]
    )
