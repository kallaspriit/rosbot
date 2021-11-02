from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            parameters=[
                join(
                    get_package_share_directory('rosbot_description'),
                    'config',
                    'slam_toolbox_mapping.yaml'
                ),
                {'use_sim_time': False}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox_localization',
            output='screen'
        )
    ])
