from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from os.path import join

# generates the robot launch description with arguments and nodes


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
                    "rosbot_system.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # get rosbot controllers configuration path
    rosbot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rosbot"),
            "config",
            "rosbot_controllers.yaml",
        ]
    )

    # setup ros2 controller manager node with robot description and controllers
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, rosbot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # setup robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # setup joint state broadcaster spawner node
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # setup diff drive controller
    diff_drive_controller_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    # get path to rviz configuration file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot.rviz"]
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
                'rosbot.config.yaml'
            ),
        ],
        remappings={('/cmd_vel', 'diff_drive_controller/cmd_vel_unstamped')},
        condition=IfCondition(LaunchConfiguration('launch_teleop')),
    )

    # setup lidar node
    lidar_node = Node(
        package='rplidar_ros2',
        executable='rplidar_scan_publisher',
        name='rplidar_scan_publisher',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_dev'),
            'serial_baudrate': 256000,
            # 'frame_id': 'laser',
            # 'frame_id': 'odom',
            'frame_id': 'lidar',
            'inverted': False,
            'angle_compensate': True
        }],
        output='screen'
    )

    # setup list of nodes to launch
    nodes = [
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        diff_drive_controller_node,
        rviz_node,
        joy_node,
        teleop_node,
        lidar_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
