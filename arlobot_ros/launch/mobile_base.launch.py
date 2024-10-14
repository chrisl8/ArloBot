import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable


# To test alone:
# source ~/ros2_ws/install/setup.zsh
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
# ros2 launch arlobot_ros mobile_base.launch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arlobot_ros',
            executable='propeller_node',
            name='arlobot',
            parameters=[
                {'lastX': EnvironmentVariable('lastX')},
                {'lastY': EnvironmentVariable('lastY')},
                {'lastHeading': EnvironmentVariable('lastHeading')},
                {"driveGeometry/trackWidth": EnvironmentVariable('trackWidth')},
                {"driveGeometry/distancePerCount": EnvironmentVariable('distancePerCount')},
                {"driveGeometry/wheelSymmetryError": EnvironmentVariable('wheelSymmetryError')},
                {"ignoreProximity": EnvironmentVariable('ignoreProximity')},
                {"ignoreCliffSensors": EnvironmentVariable('ignoreCliffSensors')},
                {"ignoreIRSensors": EnvironmentVariable('ignoreIRSensors')},
                {"ignoreFloorSensors": EnvironmentVariable('ignoreFloorSensors')},
                {"pluggedIn": EnvironmentVariable('pluggedIn')},
                {"port": EnvironmentVariable('ACTIVITY_BOARD_PORT')},
                {"baudRate": EnvironmentVariable('activityBoardbaudRate')},
                {"maxPingRangeAccepted": EnvironmentVariable('maxPingRangeAccepted')},
            ]
        ),
        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory(
        #             'arlobot_ros'), 'launch/twist_mux.launch.py')
        #     )
        # )
    ])


if __name__ == '__main__':
    generate_launch_description()
