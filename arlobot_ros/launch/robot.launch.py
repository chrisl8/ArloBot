import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch_ros.actions import Node, SetRemap, SetParametersFromFile
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# This launch file, and its child propeller_node_launch.py require many environment variables to be set.
# The script ros_start.sh does this for you.

# To test alone without running a shell script:
# source ~/ros2_ws/install/setup.bash
# RPLIDAR_USB_PORT=$(find_RPLIDAR.sh)
# RPLIDAR_BAUDRATE=256000
# ACTIVITY_BOARD_PORT=$("find_ActivityBoard.sh")
# maxPingRangeAccepted=$(jq '.maxPingRangeAccepted' "${HOME}/.arlobot/personalDataForBehavior.json")
# trackWidth=$(jq '.driveGeometry.trackWidth' "${HOME}/.arlobot/personalDataForBehavior.json")
# distancePerCount=$(jq '.driveGeometry.distancePerCount' "${HOME}/.arlobot/personalDataForBehavior.json")
# wheelSymmetryError=$(jq '.driveGeometry.wheelSymmetryError' "${HOME}/.arlobot/personalDataForBehavior.json")
# ignoreProximity=$(jq '.ignoreProximity' "${HOME}/.arlobot/personalDataForBehavior.json")
# ignoreCliffSensors=$(jq '.ignoreCliffSensors' "${HOME}/.arlobot/personalDataForBehavior.json")
# ignoreIRSensors=$(jq '.ignoreIRSensors' "${HOME}/.arlobot/personalDataForBehavior.json")
# ignoreFloorSensors=$(jq '.ignoreFloorSensors' "${HOME}/.arlobot/personalDataForBehavior.json")
# pluggedIn=$(jq '.pluggedIn' "${HOME}/.arlobot/personalDataForBehavior.json")
# activityBoardbaudRate=$(jq '.activityBoardbaudRate' "${HOME}/.arlobot/personalDataForBehavior.json")
# lastX=0.0
# lastY=0.0
# lastHeading=0.0
# ros2 launch arlobot_ros robot_launch.py

# TODO: Update these old notes for ROS2
# <!-- If you want to also view this in RViz, you must have a transform. -->
# <!-- rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map rplidar 1000 -->
# <!-- Then you can start up rviz and see the scan data -->

# <!-- If you want to play with the offset/rotation/etc. modify the parameters to match your URDF file like so:-->
# <!-- rosrun tf static_transform_publisher 0.125 0.0 0.269107 3.1 0.0 0.0 map rplidar 1000 -->
# <!-- Note that the 3.1 (approximately PI) lines it up "backwards" with the cord toward the back of the robot)-->

def generate_launch_description():
    # For robot_state_publisher
    urdf = os.path.join(get_package_share_directory('arlobot_ros'), 'arlobot.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # For teleop_twist_joy
    joy_node_config_filepath = [
        TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
        'xbox', TextSubstitution(text='.config.yaml')
    ]

    return LaunchDescription([
        # Websocket bridge for web interface with Robot.
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rosbridge_server'), 'launch/rosbridge_websocket_launch.xml')
            )
        ),
        # Robot model publisher for RVIZ, etc.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'publish_frequency': 5.0
                },
                {
                    'robot_description': robot_description,
                }
            ],
        ),
        # Propeller Activity Board interface code
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arlobot_ros'), 'launch/propeller_node.launch.py')
            )
        ),
        # TODO: Put this back.
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory(
        #             'arlobot_ros'), 'launch/arlobot_goto.launch.py')
        #     )
        # )
        # Joystick input
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                'joy_config': 'xbox',
            }]),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_node_config_filepath, {'publish_stamped_twist': False, 'enable_button': 4}],
            remappings={('/cmd_vel', '/joy_vel')},
        ),
        SetRemap(src='/cmd_vel_out', dst='/cmd_vel'),
        SetParametersFromFile(get_package_share_directory('arlobot_ros') + '/param/twist_mux_topics.yaml'),
        SetParametersFromFile(get_package_share_directory('arlobot_ros') + '/param/twist_mux_locks.yaml'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'twist_mux'), 'launch/twist_mux_launch.py')
            ),
        ),
        # Lidar Node
        Node(
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
                    # In theory this can be anywhere from 5.0 to 20.0, with 10.0 being the default.
                    # a "10Hz" san frequency is a 16 kHz sample rate according to the Slamtec A3 Datasheet here:
                    # https://www.slamtec.ai/wp-content/uploads/2023/11/LD310_SLAMTEC_rplidar_datasheet_A3M1_v1.9_en.pdf
                    # I have not tested yet to see how adjusting this affects results and/or CPU usage.
                    'scan_frequency': 10.0
                },
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
