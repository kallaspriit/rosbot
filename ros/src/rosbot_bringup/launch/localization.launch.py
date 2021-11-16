from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join, abspath
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")

    # defaults
    default_map = abspath(join(
        robot_description_path,
        "..",
        "..",
        "..",
        "..",
        "map",
        "office.yaml"
    ))
    nav2_config = join(
        robot_description_path,
        "config",
        "nav2.yaml"
    )

    # launch arguments
    autostart = LaunchConfiguration("autostart")
    map = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "autostart",
            default_value="True",
            description="Automatically start up the nav2 stack",
        ),

        DeclareLaunchArgument(
            name="map",
            default_value=default_map,
            description="Full path to map file to load"
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation time",
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                nav2_config,
                {"yaml_filename": map},
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                nav2_config,
                {"use_sim_time": use_sim_time},
            ],
            # remappings=remappings
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": ["map_server", "amcl"]}
                # {"node_names": ["amcl"]}
            ]
        ),

        # Node(
        #     parameters=[
        #         join(
        #             get_package_share_directory("rosbot_description"),
        #             "config",
        #             "slam_toolbox_localization.yaml"
        #         ),
        #         # {"use_sim_time": use_sim_time}
        #     ],
        #     package="slam_toolbox",
        #     executable="localization_slam_toolbox_node",
        #     name="slam_toolbox",
        #     output="screen"
        # ),
    ])
