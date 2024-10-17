import os
# https://docs.ros.org/en/ros2_packages/iron/api/teleop_twist_joy/standard_docs/README.html
# Based on: jazzy/share/teleop_twist_joy/launch/teleop-launch.py

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


# Push the left frontal button labeled as 'LB' to activate cmd_vel publishing.
# Move the left stick around to control the velocity.

# TODO: Does teleop_tiwst_joy_node need paramemter 'scale_angular': 1.5?

def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='joy_vel'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='0'),
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'device_id': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist, 'enable_button': 4}],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
        ),
    ])
