import launch
import launch_ros.actions
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch import LaunchDescription


# Push the left frontal button labeled as 'LB' to activate cmd_vel publishing.
# Move the left stick around to control the velocity.


def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='joy_config',
            default_value='xbox'
        ),
        launch.actions.DeclareLaunchArgument(
            name='config_filepath',
            default_value=LaunchConfiguration(
                'joy_config')
        ),
        launch.actions.DeclareLaunchArgument(
            name='joy_topic',
            default_value='joy'
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {
                    'dev': EnvironmentVariable('JOY_DEVICE')
                },
                {
                    'deadzone': 0.3
                },
                {
                    # ROS requires a repeat on the output to keep moving the robot,
                    # although this means you need a cooldown on button inputs if you use them for anything else.
                    'autorepeat_rate': 20.0
                }
            ]
        ),
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[
                {
                    'scale_angular': 1.5
                },
                {
                    'enable_button': 4
                },
                LaunchConfiguration('config_filepath')
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
