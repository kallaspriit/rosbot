from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    slam_toolbox_config_path = join(
        robot_description_path,
        "config",
        "slam_toolbox_mapping.yaml"
    )

    # defaults
    default_map = join(
        robot_description_path,
        "map",
        "office.yaml"
    )

    # launch arguments
    map = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument(
            name="map",
            default_value=default_map,
            description="Full path to map file to load"
        ),

        Node(
            parameters=[
                slam_toolbox_config_path,
                {"use_sim_time": False}
            ],
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox_localization",
            output="screen"
        )
    ])
