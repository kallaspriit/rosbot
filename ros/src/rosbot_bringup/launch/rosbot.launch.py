from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    # list of launch arguments
    declared_arguments = []

    # should we open rviz visualizer (disabled by default)
    # `ros2 launch rosbot rviz.py` on remote pc instead
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
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

    # joystick device to use
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device to use",
        )
    )

    # lidar device to use
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar_dev",
            default_value="/dev/ttyS0",
            description="Lidar device to use",
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
    robot_description = {"robot_description": robot_description_content}

    # get rosbot controllers configuration path
    diff_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_description"),
            "config",
            "diff_drive_controller.yaml",
        ]
    )

    # setup ros2 controller manager node with robot description and controllers
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, diff_drive_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # setup robot state publisher node (publishes transforms)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    # setup joint state broadcaster spawner node
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            # "--controller-manager", "/controller_manager",
        ],
    )

    # setup diff drive controller
    diff_drive_spawner_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_controller"],
        output="screen",
    )

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

    # setup lidar node
    lidar_node = Node(
        package='rplidar_ros2',
        executable='rplidar_scan_publisher',
        name='rplidar_scan_publisher',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_dev'),
            'serial_baudrate': 256000,
            'frame_id': 'base_laser',
            'inverted': False,
            'angle_compensate': True
            # 'angle_compensate': False
        }],
        output='screen'
    )

    # setup joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        # remap joy topic to allow to co-exist with another joystick
        remappings={('/joy', '/joy_local')},
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
                'rosbot_xbox_bluetooth.config.yaml'
            ),
        ],
        remappings={
            ('/joy', '/joy_local')
        },
        condition=IfCondition(LaunchConfiguration('launch_teleop')),
    )

    # setup slam toolbox node in async slam mode (builds the map)
    # https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py
    # https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml
    # slam_toolbox_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox_node',
    #     parameters=[
    #         join(
    #             get_package_share_directory('rosbot_description'),
    #             'config',
    #             'slam_toolbox_async.yaml'
    #         ),
    #         {'use_sim_time': False}
    #     ]
    # )

    # setup list of nodes to launch
    nodes = [
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        diff_drive_spawner_node,
        rviz_node,
        lidar_node,
        joy_node,
        teleop_node,
        # slam_toolbox_node
    ]

    return LaunchDescription(declared_arguments + nodes)
