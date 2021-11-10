from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    bno055_config = join(robot_description_path, "config", "bno055.yaml")

    # launch arguments
    uart_port = LaunchConfiguration("uart_port")

    # build launch description
    return LaunchDescription([

        DeclareLaunchArgument(
            "uart_port",
            default_value="/dev/ttyAMA0",
            description="Uart serial port device to use",
        ),

        # imu (used by robot_localization to make odometry more precise)
        Node(
            package="bno055",
            executable="bno055",
            parameters=[
                bno055_config,
                {"uart_port": uart_port}
            ],
        ),

        # fuses imu and odometry to produce more precise filtered odometry
        # Node(
        #     package="robot_localization",
        #     executable="ekf_node",
        #     name="robot_localization_node",
        #     output="screen",
        #     parameters=[
        #         robot_localization_config,
        #     ]
        # ),

    ])
