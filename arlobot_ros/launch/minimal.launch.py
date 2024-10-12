import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/publish_robot_model.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/mobile_base.launch.py')
            )
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
