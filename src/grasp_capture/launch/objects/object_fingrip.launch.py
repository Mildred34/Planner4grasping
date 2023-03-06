"""Launch script for spawning Franka Emika Panda into Ignition Gazebo world"""

from typing import List

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

    # List of nodes to be launched
    nodes = [
        # ros_ign_gazebo_create
        Node(
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=["-file", model, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


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
        # Object Model for Ignition Gazebo
        DeclareLaunchArgument(
            "object_type",
            default_value="coude",
            description="Name of the object type to use.",
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
                        FindPackageShare("fingrip_description"),
                        "ressources",
                        LaunchConfiguration("object_type"),
                        LaunchConfiguration("config_file_name"),
                    ]
                ),
            description="Object Config Path",
        ),
    ]