import launch_ros.actions
from launch.substitutions import EnvironmentVariable
from launch import LaunchDescription


# To test alone:
# source ~/ros2_ws/install/setup.zsh
# RPLIDAR_USB_PORT=$(find_RPLIDAR.sh) RPLIDAR_BAUDRATE=256000 ros2 launch arlobot_ros rplidar.launch.py

# TODO: Update these old notes for ROS2
# <!-- If you want to also view this in RViz, you must have a transform. -->
# <!-- rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map rplidar 1000 -->
# <!-- Then you can start up rviz and see the scan data -->

# <!-- If you want to play with the offset/rotation/etc. modify the parameters to match your URDF file like so:-->
# <!-- rosrun tf static_transform_publisher 0.125 0.0 0.269107 3.1 0.0 0.0 map rplidar 1000 -->
# <!-- Note that the 3.1 (approximately PI) lines it up "backwards" with the cord toward the back of the robot)-->

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[
                {
                    'serial_port': EnvironmentVariable('RPLIDAR_USB_PORT'),
                },
                {
                    # A3 Works at 256000, others at half this speed.
                    'serial_baudrate': EnvironmentVariable('RPLIDAR_BAUDRATE'),
                },
                {
                    'frame_id': 'rplidar'
                },
                {
                    'inverted': False,
                },
                {
                    # Angle compensation should make this work well with mapping tools https://github.com/allenh1/rplidar_ros/pull/12
                    # "With angle compensation enabled you should be able to get consistent laser scan sizes that are evenly distributed which will make slam packages happy."
                    'angle_compensate': True
                },
                {
                    'scan_mode': 'Sensitivity'
                },
                {
                    'scan_frequency': 10.0
                },
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
