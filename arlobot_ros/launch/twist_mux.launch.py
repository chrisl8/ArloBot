from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'arlobot_ros') + '/param/twist_mux_topics.yaml',
                get_package_share_directory(
                    'arlobot_ros') + '/param/twist_mux_locks.yaml'
            ],
            remappings=[
                ('/cmd_vel_out', '/cmd_vel'),
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
