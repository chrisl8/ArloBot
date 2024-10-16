import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# You must use the `xacro` command to convert the URDF stack to an xacro xml file.
# ` source ~/ros2_ws/install/setup.zsh;/opt/ros/jazzy/bin/xacro ~/ArloBot/urdf/arlo.urdf.xacro > ~/ArloBot/arlobot_ros/arlobot.urdf`

def generate_launch_description():

    urdf_file_name = 'arlobot.urdf'
    urdf = os.path.join(
        get_package_share_directory('arlobot_ros'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
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
                       output="screen",
                   )
               ])

if __name__ == '__main__':
    generate_launch_description()
