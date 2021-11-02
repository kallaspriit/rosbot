from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from os.path import join


def generate_launch_description():
    # list of launch arguments
    declared_arguments = []

    # should we open rviz visualizer (enabled by default)
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz automatically with the launch file",
        )
    )

    # should we start teleop (enabled by default)
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_teleop",
            default_value="true",
            description="Start joystick teleop automatically with the launch file",
        )
    )

    # should we use simulation time
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    # joystick device to use
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device to use",
        )
    )

    # gazebo simulation world to use
    default_gazebo_world = join(
        get_package_share_directory('rosbot_description'),
        'world',
        'office.sdf'
        # 'simple.sdf'
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=default_gazebo_world,
            description="Gazebo simulation world to use",
        )
    )

    # get robot description from urdf xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "urdf",
                    "rosbot.urdf.xacro",
                ]
            ),
        ]
    )

    # resolve directories used in the configuration
    bringup_dir = get_package_share_directory('rosbot_bringup')
    launch_dir = join(bringup_dir, 'launch')

    # start gazebo server with simulated world
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            LaunchConfiguration("world")
        ],
        cwd=[launch_dir],
        output="screen"
    )

    # start gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen"
    )

    # spawn gazebo rosbot robot
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rosbot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # setup robot state publisher node (publishes transforms)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content
            }
        ],
    )

    # robot localization node
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[
    #         join(
    #             get_package_share_directory('rosbot_description'),
    #             'config',
    #             'ekf.yaml'
    #         ),
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ],
    # )

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
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # setup joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('launch_teleop')),
    )

    # setup teleop node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            join(
                get_package_share_directory('teleop_twist_joy'),
                'config',
                'rosbot_xbox_usb.config.yaml'
            ),
        ],
        condition=IfCondition(LaunchConfiguration('launch_teleop')),
    )

    # setup slam toolbox node in async slam mode (builds the map)
    # https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py
    # https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_node',
        parameters=[
            join(
                get_package_share_directory('rosbot_description'),
                'config',
                'slam_toolbox_mapping.yaml'
            ),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # setup slam toolbox node in lifelong slam mode (builds the map)
    # https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/lifelong_launch.py
    # https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_lifelong.yaml
    # slam_toolbox_node = Node(
    #     package='slam_toolbox',
    #     executable='lifelong_slam_toolbox_node',
    #     name='slam_toolbox_node',
    #     parameters=[
    #         join(
    #             get_package_share_directory('rosbot_description'),
    #             'config',
    #             'slam_toolbox_lifelong.yaml'
    #         ),
    #         # not used?
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ]
    # )

    # setup list of nodes to launch
    nodes = [
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity_node,
        robot_state_publisher_node,
        # robot_localization_node,
        rviz_node,
        joy_node,
        teleop_node,
        slam_toolbox_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
