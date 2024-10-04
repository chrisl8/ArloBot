import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'arlobot_ros') + '/param/twist_mux_topics.yaml',
                get_package_share_directory(
                    'arlobot_ros') + '/param/twist_mux_locks.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
