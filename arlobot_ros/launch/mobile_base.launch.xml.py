import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='arlobot_ros',
            executable='propellerbot_node.py',
            name='arlobot',
            parameters=[
                {
                    'bonus': 'false'
                },
                {
                    'update_rate': '30.0'
                },
                os.environ.get('HOME') + '/.arlobot/arlobot.yaml'
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/includes/twist_mux.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
