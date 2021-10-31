from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # list of launch arguments
    declared_arguments = []

    # should we use simulation time
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )

    # setup simultaneous localization and mapping node
    slam_toolbox_node = Node(
        parameters=[
            join(
                get_package_share_directory('rosbot_description'),
                'config',
                'slam_toolbox_async.yaml'
            ),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    nodes = [
        slam_toolbox_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
