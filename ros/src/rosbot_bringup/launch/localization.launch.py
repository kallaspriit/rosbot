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
    localization_config = join(
        robot_description_path,
        "config",
        "localization.yaml"
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
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static")
    ]
    configured_params = RewrittenYaml(
        source_file=localization_config,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    # lifecycle_nodes = [
    #     "map_server",
    #     "amcl",
    #     "controller_server",
    #     "planner_server",
    #     "recoveries_server",
    #     "bt_navigator",
    #     "waypoint_follower",
    # ]
    lifecycle_nodes = [
        "map_server",
        "amcl",
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

        # Node(
        #     package="nav2_controller",
        #     executable="controller_server",
        #     output="screen",
        #     parameters=[configured_params],
        #     remappings=remappings
        # ),

        # Node(
        #     package="nav2_planner",
        #     executable="planner_server",
        #     name="planner_server",
        #     output="screen",
        #     parameters=[configured_params],
        #     remappings=remappings
        # ),

        # Node(
        #     package="nav2_recoveries",
        #     executable="recoveries_server",
        #     name="recoveries_server",
        #     output="screen",
        #     parameters=[configured_params],
        #     remappings=remappings
        # ),

        # Node(
        #     package="nav2_bt_navigator",
        #     executable="bt_navigator",
        #     name="bt_navigator",
        #     output="screen",
        #     parameters=[configured_params],
        #     remappings=remappings
        # ),

        # Node(
        #     package="nav2_waypoint_follower",
        #     executable="waypoint_follower",
        #     name="waypoint_follower",
        #     output="screen",
        #     parameters=[configured_params],
        #     remappings=remappings
        # ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[configured_params],
            output="screen",
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[configured_params],
            remappings=remappings
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
