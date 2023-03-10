#!/usr/bin/env -S ros2 launch
"""Launch C++ example for following a target"""

from os import path
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "fingrip_moveit_config"
    robot_type = LaunchConfiguration("robot_type")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")
    object_type = LaunchConfiguration("object_type")

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "name:=",
            robot_type,
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(moveit_config_package),
                    "srdf",
                    "fingrip.srdf.xacro",
                ]
            ),
            " ",
            "name:=",
            robot_type,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": _robot_description_semantic_xml
    }

    # List of included launch descriptions
    launch_descriptions = [
        # Launch world with robot (configured for this example)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("grasp_capture"),
                        "launch",
                        "create_planning_scene.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("world_type", "follow_target"),
                ("robot_type", robot_type),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("ign_verbosity", ign_verbosity),
                ("log_level", log_level),
                ("object_type", object_type),
            ],
        ),
    ]

    # List of nodes to be launched
    # Create Planning scene
    nodes = [
        Node(
            package="grasp_capture",
            executable="get_expert_pose",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                use_sim_time,
                object_type,
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)

def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="fingrip_description", # A crér
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "fingrip.urdf.xacro"),
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # Robot selection
        DeclareLaunchArgument(
            "robot_type",
            default_value="fingrip",
            description="Name of the robot to use.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("grasp_capture"),
                "rviz",
                "grasp_capture.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true", # if true doesn't work, block the node
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="2",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
        DeclareLaunchArgument(
            "object_type",
            default_value="coude",
            description="Object type to use for simulation.",
        ),
    ]
