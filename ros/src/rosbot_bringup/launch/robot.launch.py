from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    diff_drive_controller_config = join(
        robot_description_path, "config", "diff_drive_controller.yaml"
    )
    bno055_config = join(robot_description_path, "config", "bno055.yaml")
    robot_localization_config = join(
        robot_description_path,
        "config",
        "ekf.yaml",
    )
    teleop_config = join(
        robot_description_path,
        "config",
        "teleop_bluetooth.yaml"
    )

    # launch arguments
    launch_teleop = LaunchConfiguration("launch_teleop")
    joy_dev = LaunchConfiguration("joy_dev")
    lidar_dev = LaunchConfiguration("lidar_dev")

    # get robot description from urdf xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    robot_description_path,
                    "urdf",
                    "rosbot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # build launch description
    return LaunchDescription([

        DeclareLaunchArgument(
            "launch_teleop",
            default_value="true",
            description="Start joystick teleop automatically with the launch file",
        ),

        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device to use",
        ),

        DeclareLaunchArgument(
            "lidar_dev",
            default_value="/dev/ttyS0",
            description="Lidar device to use",
        ),

        # ros2 control used by differential drive
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, diff_drive_controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),

        # robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
            remappings=[
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ],
        ),

        # joint state broadcaster
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_state_broadcaster"],
        ),

        # differential drive controller
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["diff_drive_controller"],
            output="screen",
        ),

        # lidar
        Node(
            package="rplidar_ros2",
            executable="rplidar_scan_publisher",
            name="rplidar_scan_publisher",
            parameters=[{
                "serial_port": lidar_dev,
                "serial_baudrate": 256000,
                "frame_id": "base_laser",
                "inverted": False,
                # "angle_compensate": False
                "angle_compensate": True
            }],
            output="screen"
        ),

        # imu (used by robot_localization to make odometry more precise)
        Node(
            package="bno055",
            executable="bno055",
            parameters=[bno055_config],
        ),

        # fuses imu and odometry to produce more precise filtered odometry
        # Node(
        #     package="robot_localization",
        #     executable="ekf_node",
        #     name="robot_localization_node",
        #     output="screen",
        #     parameters=[robot_localization_config]
        # ),

        # joystick, used by onboard teleop
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node_robot",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }],
            # remap joy topic to allow to co-exist with another joystick
            remappings={
                ("/joy", "/joy_robot")
            },
            condition=IfCondition(launch_teleop),
        ),

        # onboard teleop
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_node_robot",
            parameters=[teleop_config],
            remappings={
                ("/joy", "/joy_robot")
            },
            condition=IfCondition(launch_teleop),
        ),

        # slam toolbox to map environemnt (usually run on remote pc)
        # Node(
        #     package="slam_toolbox",
        #     executable="async_slam_toolbox_node",
        #     name="slam_toolbox_node",
        #     parameters=[
        #         join(
        #             get_package_share_directory("rosbot_description"),
        #             "config",
        #             "slam_toolbox_mapping.yaml"
        #         ),
        #         {"use_sim_time": False}
        #     ]
        # ),

        # TODO map server node?
        # TODO AMCL node?

    ])
