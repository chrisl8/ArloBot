import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arlobot_ros',
            executable='propellerbot_node',
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
        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory(
        #             'arlobot_ros'), 'launch/includes/twist_mux.launch.py')
        #     )
        # )
    ])


if __name__ == '__main__':
    generate_launch_description()
