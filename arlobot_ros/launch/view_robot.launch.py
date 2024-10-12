from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz',
            executable='rviz',
            name='rviz'
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
