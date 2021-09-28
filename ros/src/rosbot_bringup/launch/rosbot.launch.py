from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# generates the robot launch description with arguments and nodes


def generate_launch_description():
    # list of launch arguments
    declared_arguments = []

    # should we open rviz visualizer
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="start RViz automatically with the launch file",
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
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    nodes = [
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        diff_drive_controller_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
