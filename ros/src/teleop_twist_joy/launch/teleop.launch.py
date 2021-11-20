from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch


def generate_launch_description():
    # launch arguments
    cmd_vel = LaunchConfiguration("cmd_vel")
    joy_config = LaunchConfiguration("joy_config")
    joy_dev = LaunchConfiguration("joy_dev")
    config_filepath = LaunchConfiguration("config_filepath")

    # build launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            "cmd_vel", default_value="/cmd_vel"
        ),

        DeclareLaunchArgument(
            "joy_config", default_value="xbox_usb"
        ),

        DeclareLaunchArgument(
            "joy_dev", default_value="/dev/input/js0"
        ),

        DeclareLaunchArgument(
            "config_filepath", default_value=[
                TextSubstitution(
                    text=join(
                        get_package_share_directory("teleop_twist_joy"),
                        "config",
                        ""
                    )
                ),
                joy_config,
                TextSubstitution(text=".config.yaml")
            ]
        ),

        # joystick, used by remote teleop
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node_remote",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }],
            # remap joy topic to allow to co-exist with another joystick
            remappings={
                ("/joy", "/joy_remote")
            },
        ),

        # remote teleop
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[config_filepath],
            remappings={
                ("/joy", "/joy_remote"),
                ("/cmd_vel", cmd_vel),
            },
        ),
    ])
