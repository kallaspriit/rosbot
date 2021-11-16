from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")

    # defaults
    default_map = join(
        robot_description_path,
        "map",
        "office.yaml"
    )
    nav2_config = join(
        robot_description_path,
        "config",
        "nav2.yaml"
    )

    # launch arguments
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")
    map = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # config
    param_substitutions = {
        "yaml_filename": map,
        "use_sim_time": use_sim_time,
        "autostart": autostart
    }
    configured_params = RewrittenYaml(
        source_file=nav2_config,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    lifecycle_nodes = [
        "map_server"
    ]

    return LaunchDescription([
        # set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            "namespace", default_value="",
            description="Top-level namespace"
        ),

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
            parameters=[configured_params],
            output="screen",
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": lifecycle_nodes}
            ]
        ),
    ])
