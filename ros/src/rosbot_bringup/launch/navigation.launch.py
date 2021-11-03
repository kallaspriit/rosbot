from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    nav2_bringup_path = get_package_share_directory("nav2_bringup")
    nav2_launch_path = join(nav2_bringup_path, "launch")
    # nav2_launch_script = join(nav2_launch_path, "bringup_launch.py")
    nav2_launch_script = join(nav2_launch_path, "navigation_launch.py")
    # nav2_bt_path = get_package_share_directory("nav2_bt_navigator")

    # defaults
    # default_behavior_tree_xml = join(
    #     nav2_bt_path,
    #     "behavior_trees",
    #     "navigate_w_replanning_and_recovery.xml"
    # )
    default_behavior_tree_xml = join(
        robot_description_path,
        "config",
        "nav2_behaviour_tree.xml"
    )

    # this is only used in slam mode
    # default_map = join(
    #     robot_description_path,
    #     "..",
    #     "..",
    #     "map",
    #     "office.yaml"
    # )
    default_map = "/home/labor/rosbot/ros/map/office.yaml"
    default_nav2_config = join(robot_description_path, "config", "nav2.yaml")
    # default_nav2_config = join(nav2_bringup_path, "params", "nav2_params.yaml")

    # launch arguments
    autostart = LaunchConfiguration("autostart")
    map = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    slam = LaunchConfiguration("slam")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # build launch description
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
            "params_file",
            default_value=default_nav2_config,
            description="Nav2 stack parameters file to use",
        ),
        DeclareLaunchArgument(
            name="slam",
            # default_value="True",
            default_value="False",
            description="Should we run with baked-in slam demo mode",
        ),
        DeclareLaunchArgument(
            name="default_bt_xml_filename",
            default_value=default_behavior_tree_xml,
            description="Full path to the behavior tree xml file to use"
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace to use",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="False",
            description="Whether to apply a namespace to the navigation stack"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation time",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_script),
            launch_arguments={
                "autostart": autostart,
                "map": map,
                "params_file": params_file,
                "slam": slam,
                "namespace": namespace,
                "use_namespace": use_namespace,
                "default_bt_xml_filename": default_bt_xml_filename,
                "use_sim_time": use_sim_time,
            }.items()
        )
    ])
