import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
    # TODO: enable RVIZ
#         launch_ros.actions.Node(
#             package='rviz',
#             executable='rviz',
#             name='rviz'
#         ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/includes/publish_robot_model.xml.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
