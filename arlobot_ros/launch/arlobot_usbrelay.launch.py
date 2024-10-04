import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='arlobot_ros',
            executable='arlobot_usbrelay.py',
            name='arlobot_usbrelay',
            parameters=[
                get_package_share_directory(
                    'arlobot_ros') + '/param/usbrelay.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
