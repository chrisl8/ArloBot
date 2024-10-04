import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='arlobot_model',
            default_value=os.environ.get('ARLOBOT_MODEL', 'default')
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdf_file',
            default_value=launch.substitutions.LaunchConfiguration(
                'arlobot_model')
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': 'None'
                },
                {
                    'publish_frequency': 5.0
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
