from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # list of launch arguments
    declared_arguments = []

    # get path to rviz configuration file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "rviz", "rosbot.rviz"]
    )

    # setup rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
