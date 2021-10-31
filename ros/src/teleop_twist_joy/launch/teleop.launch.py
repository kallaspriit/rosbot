from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch


def generate_launch_description():
    cmd_vel = launch.substitutions.LaunchConfiguration('cmd_vel')
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration(
        'config_filepath')

    return LaunchDescription([
        DeclareLaunchArgument(
            # diff drive defaults to /diff_drive_controller/cmd_vel_unstamped but modified version uses /cmd_vel
            # 'cmd_vel', default_value='/diff_drive_controller/cmd_vel_unstamped'
            'cmd_vel', default_value='/cmd_vel'
        ),

        DeclareLaunchArgument(
            'joy_config', default_value='rosbot_xbox_usb'
        ),

        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0'
        ),

        DeclareLaunchArgument(
            'config_filepath', default_value=[
                launch.substitutions.TextSubstitution(
                    text=join(
                        get_package_share_directory('teleop_twist_joy'),
                        'config',
                        ''
                    )
                ),
                joy_config,
                launch.substitutions.TextSubstitution(text='.config.yaml')
            ]
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            # remap joy topic to allow to co-exist with another joystick
            remappings={('/joy', '/joy_teleop')},
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath],
            remappings={
                ('/cmd_vel', cmd_vel),
                ('/joy', '/joy_teleop')
            },
        ),
    ])
