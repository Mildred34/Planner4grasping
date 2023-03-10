"""Launch script for spawning Franka Emika Panda into Ignition Gazebo world"""
import os
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    model = LaunchConfiguration("object_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    config_file_name = LaunchConfiguration("config_file_name")
    config_path = LaunchConfiguration("config_path")
    object_type = LaunchConfiguration("object_type")
    object_offset_pos = LaunchConfiguration("object_offset_pos")
    object_offset_rot = LaunchConfiguration("object_offset_rot")
    object_model_file = LaunchConfiguration("object_model_file")

    # Load Config Files
    config = load_yaml('grasp_capture',"config/object_config.yaml")
    config = config[config["object_type"]]

    # List of nodes to be launched
    nodes = [
        # ros_ign_gazebo_create
        Node(
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=["-file", model,"-x", str(config["offset_pos"][0]),"-y",str(config["offset_pos"][1]),"-z",str(config["offset_pos"][2]), \
                       "-R",str(config["offset_rot"][0]),"-P",str(config["offset_rot"][1]),"-Y",str(config["offset_rot"][2]),"--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_gz_bridge (clock -> ROS 2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                "/gazebo/entity/position@ros_gz_interfaces/srv/SetEntityPose",
                "--ros-args",
                "--log-level",
                log_level,
            ],
        )
    ]

    return LaunchDescription(declared_arguments + nodes)

def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
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
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
        # Object Config Name
        DeclareLaunchArgument(
            "config_file_name",
            default_value = "object_config.yaml",
            description="Name of the config file.",
        ),
        # Object Config Path
        DeclareLaunchArgument(
            "config_path",
            default_value = PathJoinSubstitution(
                    [
                        FindPackageShare("grasp_capture"),
                        "config",
                        LaunchConfiguration("config_file_name"),
                    ]
                ),
            description="Object Config Path",
        ),
        # Object Model for Ignition Gazebo
        DeclareLaunchArgument(
            "object_type",
            default_value="coude",
            description="Name of the object type to use.",
        ),
        DeclareLaunchArgument(
            "object_model_file",
            default_value="coude100_rototrans_singleface_simplified.stl",
            description="model file name",
        ),
        DeclareLaunchArgument(
            "object_offset_pos",
            default_value="[0,0,0]",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "object_offset_rot",
            default_value="[0,0,0]",
            description="If true, use simulated clock.",
        ),
    ]