import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='scanTopicSource',
            default_value=os.environ.get('SCAN_TOPIC_SOURCE', 'rplidar')
        ),
        launch.actions.DeclareLaunchArgument(
            name='loadJoystick',
            default_value=os.environ.get('HAS_XBOX_JOYSTICK', 'false')
        ),
        launch.actions.DeclareLaunchArgument(
            name='loadRPLIDAR',
            default_value=os.environ.get('HAS_RPLIDAR', 'true')
        ),
        launch.actions.IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rosbridge_server'), 'launch/rosbridge_websocket_launch.xml')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/minimal.launch.py')
            )
        ),
        # TODO: PUt this back.
        # launch.actions.IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory(
        #             'arlobot_ros'), 'launch/arlobot_goto.launch.py')
        #     )
        # )
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/xbox360_teleop.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/rplidar.launch.py')
            )
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
