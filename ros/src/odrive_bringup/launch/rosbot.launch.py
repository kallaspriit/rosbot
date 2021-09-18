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

    # should we enable the left wheel joint
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_left_wheel_joint",
            default_value="true",
        )
    )

    # should we enable the right wheel joint
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_right_wheel_joint",
            default_value="true",
        )
    )

    # left wheel joint controller, defaults to velocity controller
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_wheel_joint_controller",
            default_value="left_wheel_joint_velocity_controller",
        )
    )

    # right wheel joint controller, defaults to velocity controller
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_wheel_joint_controller",
            default_value="right_wheel_joint_velocity_controller",
        )
    )

    # should we open rviz visualizer
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="start RViz automatically with the launch file",
        )
    )

    # get launch argument values
    enable_left_wheel_joint = LaunchConfiguration("enable_left_wheel_joint")
    enable_right_wheel_joint = LaunchConfiguration("enable_right_wheel_joint")
    # left_wheel_joint_controller = LaunchConfiguration(
    #     "left_wheel_joint_controller"
    # )
    # right_wheel_joint_controller = LaunchConfiguration(
    #     "right_wheel_joint_controller"
    # )

    # get robot description from urdf xacro file
    # TODO: add stuff from diffbot_system.urdf.xacro to rosbot.urdf.xacro
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
            " ",
            "enable_left_wheel_joint:=",
            enable_left_wheel_joint,
            " ",
            "enable_right_wheel_joint:=",
            enable_right_wheel_joint,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # get odrive controllers configuration
    # odrive_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("odrive_bringup"),
    #         "config",
    #         "odrive_controllers.yaml",
    #     ]
    # )

    # get diffbot diff controllers configuration
    # diffbot_diff_drive_controller = PathJoinSubstitution(
    #     [
    #         FindPackageShare("odrive_bringup"),
    #         "config",
    #         "diffbot_diff_drive_controller.yaml",
    #     ]
    # )

    # get rosbot controllers configuration path
    rosbot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_bringup"),
            "config",
            "rosbot_controllers.yaml",
        ]
    )

    # setup ros2 controller manager node with robot description and controllers
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # parameters=[robot_description, odrive_controllers, diffbot_diff_drive_controller],
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

    # setup left wheel joint controller spawner node
    # left_wheel_controller_node = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=[left_wheel_joint_controller, "-c", "/controller_manager"],
    #     condition=IfCondition(enable_left_wheel_joint),
    # )

    # setup right wheel joint controller spawner node
    # right_wheel_controller_node = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=[right_wheel_joint_controller, "-c", "/controller_manager"],
    #     condition=IfCondition(enable_right_wheel_joint),
    # )

    # setup diffbot base controller
    diffbot_base_controller_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller"],
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
        # left_wheel_controller_node,
        # right_wheel_controller_node,
        diffbot_base_controller_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
