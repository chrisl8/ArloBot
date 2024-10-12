from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arlobot_ros',
            executable='arlobot_teleop_key.py',
            name='arlobot_teleop_keyboard',
            output='screen'
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
