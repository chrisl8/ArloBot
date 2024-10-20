# Using Black: https://github.com/ambv/black
# pylint: disable=line-too-long
# Software License Agreement (BSD License)
#
# Author: Christen Lofland https://github.com/chrisl8
# URL: https://github.com/chrisl8/ArloBot
#
# Derived from \opt\ros\hydro\lib\create_node\turtlebot_node.py
# This is based on turtlebot_node adapted to run on a Propeller Activity Board based ArloBot
#
# Special thanks to arduino.py by Dr. Rainer Hessmer
# https://code.google.com/p/drh-robotics-ros/
#
# NOTE: This script requires parameters to be loaded from ~/.arlobot/arlobot.yaml!

import rclpy
from rclpy.node import Node
import tf2_ros
from math import sin, cos
import time
import os
import subprocess

from geometry_msgs.msg import Quaternion, TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from arlobot_interfaces.msg import ArloStatus, ArloSafety, ArloButtons
from arlobot_interfaces.srv import ToggleLED

from .checkPropellerCodeVersionNumber import checkPropellerCodeVersionNumber
from .PropellerSerialInterface import PropellerSerialInterface
from .PropellerSerialDataPacketTypes import PropellerSerialDataPacketTypes
from .OdomStationaryBroadcaster import OdomStationaryBroadcaster

rclpy.init()
node = rclpy.create_node("arlobot")


class PropellerComm(Node):
    """
    Helper class for communicating with a Propeller board over serial port
    """

    def __init__(self, args=None):
        # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        self.r = node.create_rate(1)  # 1hz refresh rate
        self._safeToGo = True  # Use arlobot_safety to set this
        self._SafeToOperate = True  # Use arlobot_safety to set this
        self._unPlugging = True  # Used for when arlobot_safety tells us to "UnPlug"!
        self._wasUnplugging = (
            False  # Track previous unplugging status for motor control
        )
        self._serialAvailable = False
        self._odometry_broadcast_timeout = 0
        self._odometry_broadcast_timeout_max = 2
        self._broadcast_static_odometry = False
        self._leftMotorPower = False
        self._rightMotorPower = False
        # Store last x, y and heading for reuse when we reset
        # I took off the ~, because that was causing these to reset to default on every restart
        # even if roscore was still up.
        # https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/#Set_default_values
        node.declare_parameters(
            namespace='',
            parameters=[
                ('lastX', 0.0),
                ('lastY', 0.0),
                ('lastHeading', 0.0),
                ("driveGeometry/trackWidth", 0.403),
                ("driveGeometry/distancePerCount", 0.00338),
                ("driveGeometry/wheelSymmetryError", 1.0),
                ("ignoreProximity", False),
                ("ignoreCliffSensors", False),
                ("ignoreIRSensors", False),
                ("ignoreFloorSensors", False),
                ("pluggedIn", False),  # No-longer used, but the Propeller board expects it.
                ("port", "/dev/ttyUSB1"),
                ("baudRate", 115200),
                ("maxPingRangeAccepted", 0.5),
            ]
        )

        self.lastX = node.get_parameter("lastX").get_parameter_value().double_value
        self.lastY = node.get_parameter("lastY").get_parameter_value().double_value
        self.lastHeading = node.get_parameter("lastHeading").get_parameter_value().double_value
        self.alternate_heading = self.lastHeading

        # Store the data from ROS for comparison later
        self._settings_from_ros = {}
        self._updateSettingsFromROS()

        # Store the incoming data from the Propeller board
        self._config_from_propeller = {
            "trackWidth": 0.0,
            "distancePerCount": 0.0,
            "wheelSymmetryError": 1.0,
            # Set to opposites of self._settings
            # to ensure _settingsUpdateRequired stays true
            # until we get the data from the board.
            "ignoreProximity": not self._settings_from_ros["ignoreProximity"],
            "ignoreCliffSensors": not self._settings_from_ros["ignoreCliffSensors"],
            "ignoreIRSensors": not self._settings_from_ros["ignoreIRSensors"],
            "ignoreFloorSensors": not self._settings_from_ros["ignoreFloorSensors"],
            "pluggedIn": not self._settings_from_ros["pluggedIn"],
        }

        self._ledInputData_from_propeller = []
        self._ledRequestedState_from_ROS = []

        # Subscriptions
        node.create_subscription(Twist, "cmd_vel", self._handle_velocity_command, 1)
        # NOTE: Keep the cmd_vel subscription queue_size to 1, because we never want to allow a backlog of twist commands. Always just send the most recent one.
        # Otherwise strange behavior occurs, even the robot responding to commands very late, like stopping and then starting a command given moments ago!
        node.create_subscription(
            ArloSafety,
            "arlobot_safety/safetyStatus",
            self._safety_shutdown,
            1,
        )  # Safety Shutdown

        # Publishers
        # for publishing PIR status
        # self._pirPublisher = node.create_publisher(Bool, "~pirState", 1)
        self._arlo_status_publisher = node.create_publisher(ArloStatus, "arlo_status", 1)
        self._buttons_publisher = node.create_publisher(ArloButtons, "buttons", 1)

        # IF the Odometry Transform is done with the robot_pose_ekf do not publish it,
        # but we are not using robot_pose_ekf, because it does nothing for us if you don't have a full IMU!
        # REMOVE this line if you use robot_pose_ekf
        # https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html
        self._OdometryTransformBroadcaster = tf2_ros.TransformBroadcaster(node)

        self._OdometryPublisher = node.create_publisher(Odometry, "odom", 1)

        # We don't need to broadcast a transform, as it is static and contained within the URDF files
        # self._SonarTransformBroadcaster = tf2_ros.TransformBroadcaster()
        self._UltraSonicPublisher = node.create_publisher(LaserScan, "ultrasonic_scan", 1)
        self._InfraredPublisher = node.create_publisher(LaserScan, "infrared_scan", 1)

        # For Camera Joint Testing
        # self._JointStatePublisher = node.create_publisher(JointState, "joint_states", 1)

        # Create a ROS service that can be called
        # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

        node.create_service(ToggleLED, "ToggleLED", self._toggleLED)

        # TODO: Get port from ACTIVITY_BOARD_PORT environment variable

        # You can use the ~/dev_ws/src/ArloBot/scripts/find_ActivityBoard.sh script to find this, and
        # You can set it by running this before starting this:
        # rosparam set /arlobot/port $(~/dev_ws/src/ArloBot/scripts/find_ActivityBoard.sh)
        port = node.get_parameter("port").get_parameter_value().string_value
        baud_rate = node.get_parameter("baudRate").get_parameter_value().integer_value

        node.get_logger().info(
            "Starting with serial port: " + port + ", baud rate: " + str(baud_rate)
        )

        self.serialInterface = PropellerSerialInterface(
            self._propellerReadyResponseFunction,
            self._propellerOdomDataHandler,
            self._propellerConfigDataHandler,
            self.TestDataResponseFunction,
            self.forwardTextToRosLog,
            port,
            baud_rate,
        )
        self.dataTypes = PropellerSerialDataPacketTypes()

        self._OdomStationaryBroadcaster = OdomStationaryBroadcaster(
            self._broadcast_static_odometry_info,
            self.forwardTextToRosLog,
        )

    def forwardTextToRosLog(self, data):
        data = str(data)
        if "error" in data.lower():
            node.get_logger().warn(data)
        elif "Serial Write Delay:" in data:
            node.get_logger().debug(data)
        elif "Good Packet Delay:" in data:
            node.get_logger().debug(data)
        else:
            node.get_logger().info(data)

    def TestDataResponseFunction(self, data):
        node.get_logger().info("Test Packet Received: " + str(data))

    def _updateSettingsFromROS(self):
        self._settings_from_ros["trackWidth"] = node.get_parameter(
            "driveGeometry/trackWidth").get_parameter_value().double_value
        self._settings_from_ros["distancePerCount"] = node.get_parameter(
            "driveGeometry/distancePerCount").get_parameter_value().double_value
        self._settings_from_ros["wheelSymmetryError"] = node.get_parameter(
            "driveGeometry/wheelSymmetryError").get_parameter_value().double_value
        self._settings_from_ros["ignoreProximity"] = node.get_parameter(
            "ignoreProximity").get_parameter_value().bool_value
        self._settings_from_ros["ignoreCliffSensors"] = node.get_parameter(
            "ignoreCliffSensors").get_parameter_value().bool_value
        self._settings_from_ros["ignoreIRSensors"] = node.get_parameter(
            "ignoreIRSensors").get_parameter_value().bool_value
        self._settings_from_ros["ignoreFloorSensors"] = node.get_parameter(
            "ignoreFloorSensors").get_parameter_value().bool_value
        self._settings_from_ros["pluggedIn"] = node.get_parameter("pluggedIn").get_parameter_value().bool_value

    def _safety_shutdown(self, status):
        """
        Prevent sending twist commands if the SafeToOperate topic goes false.
        Set unPlugging variable to allow for safe unplug operation.
        """
        self.forwardTextToRosLog("_safety_shutdown")
        self._unPlugging = status.unplugging
        self._SafeToOperate = status.safe_to_operate
        self._safeToGo = status.safe_to_go

    # noinspection Duplicates
    def _propellerOdomDataHandler(self, data):
        """
        Broadcast all data from propeller monitored sensors on the appropriate topics.
        """

        now = node.get_clock().now().to_msg()

        self._odometry_broadcast_timeout = 0
        self._broadcast_static_odometry = False

        # Check for short lines, though this should never happen, as the serial receiver checks this.
        if (
                len(data) < 17
        ):  # Just discard short/long lines, increment this as lines get longer
            node.get_logger().warn("Short odometry line from Propeller board: " + str(len(data)))
            return

        # Get odometry and publish it
        x = data[0]
        y = data[1]
        # Odometry based heading
        theta = data[2]
        # Gyro based heading
        alternate_theta = data[3]
        # On ArloBot odometry derived heading works best.

        vx = data[4]
        omega = data[5]

        # This robot has no vy or vz (velocity y or velocity z) because it can only move forward,
        # and backward. It cannot move sideways or up and down.

        # For Debugging:
        # node.get_logger().warn(
        #     "x: "
        #     + str(x)
        #     + " y: "
        #     + str(y)
        #     + " theta: "
        #     + str(theta)
        #     + " vx: "
        #     + str(vx)
        #     + " omega: "
        #     + str(omega)
        # )

        self._broadcast_odometry(x, y, theta, vx, omega, now)

        # Debugging speed out of sync with actual position changes
        # node.get_logger().warn("vx: " + str(vx) + " omega: " + str(omega))
        # if self.lastHeading == theta and abs(vx) > 0:
        #     node.get_logger().warn("No movement recorded but vx = " + str(vx))
        #     node.get_logger().warn(
        #         str(self.lastX)
        #         + " "
        #         + str(x)
        #         + " "
        #         + str(self.lastY)
        #         + " "
        #         + str(y)
        #         + " "
        #         + str(self.lastHeading)
        #         + " "
        #         + str(theta)
        #         + " "
        #         + str(self.alternate_heading)
        #         + " "
        #         + str(alternate_theta)
        #         + " "
        #         + str(vx)
        #         + " "
        #         + str(omega)
        #     )

        # Save last X, Y and Heading for reuse if we have to reset:
        self.lastX = x
        self.lastY = y
        self.lastHeading = theta
        self.alternate_heading = alternate_theta

        # Get and publish other telemetry data
        arlo_status = ArloStatus()
        # Order from ROS Interface for ArloBot.c
        left_motor_voltage = (15 / 4.69) * data[6]
        right_motor_voltage = (15 / 4.69) * data[7]
        arlo_status.robot_battery_level = 12.0
        if left_motor_voltage < 1:
            arlo_status.left_motor_power = False
        else:
            arlo_status.left_motor_power = True
            arlo_status.robot_battery_level = left_motor_voltage
        self._leftMotorPower = arlo_status.left_motor_power
        if right_motor_voltage < 1:
            arlo_status.right_motor_power = False
        else:
            arlo_status.right_motor_power = True
            arlo_status.robot_battery_level = right_motor_voltage
        self._rightMotorPower = arlo_status.right_motor_power
        # 11.6 volts is the cutoff for an SLA battery.
        if arlo_status.robot_battery_level < 12:
            arlo_status.robot_battery_low = True
        else:
            arlo_status.robot_battery_low = False

        arlo_status.abd_speed_limit = data[8]
        arlo_status.abd_reverse_speed_limit = data[9]
        arlo_status.heading = self.lastHeading
        arlo_status.gyro_heading = self.alternate_heading
        arlo_status.min_distance_sensor = data[10]
        arlo_status.safe_to_proceed = True if data[11] == 1 else False
        arlo_status.safe_to_recede = True if data[12] == 1 else False
        arlo_status.escaping = True if data[13] == 1 else False
        arlo_status.cliff = True if data[14] == 1 else False
        arlo_status.floor_obstacle = True if data[15] == 1 else False

        self._arlo_status_publisher.publish(arlo_status)

        # Get and publish sensor data

        # Joint State for Turtlebot stack
        # Note without this transform publisher the wheels will
        # be white, stuck at 0, 0, 0 and RVIZ will tell you that
        # there is no transform from the wheel_links to the base_
        # However, instead of publishing a stream of pointless transforms,
        # I have made the joint static in the URDF like this:
        # create.urdf.xacro:
        # <joint name="right_wheel_joint" type="fixed">
        # NOTE This may prevent Gazebo from working with this model
        # Here is the old Joint State in case you want to restore it:
        # js = JointState(name=["left_wheel_joint", "right_wheel_joint", "front_castor_joint", "back_castor_joint"],
        # position=[0, 0, 0, 0], velocity=[0, 0, 0, 0], effort=[0, 0, 0, 0])
        # js.header.stamp = ros_now
        # self.joint_states_pub.publish(js)

        # Test publishing the camera_rgb_joint so that we can adjust it dynamically until we get it right:

        # js = JointState(
        #     name=["camera_rgb_joint"],
        #     position=[0, 1, 0, 0],
        #     velocity=[0, 0, 0, 0],
        #     effort=[0, 0, 0, 0],
        # )
        # js.header.stamp = ros_now

        # js = JointState()
        # # js.header = Header()
        # js.header.stamp = rclpy.Time.now()
        # js.header.frame_id = "camera_rgb_joint"
        # js.name = ["camera_rgb_joint"]
        # js.position = [3]
        # self._JointStatePublisher.publish(js)

        # The purpose of this is two fold:
        # 1. It REALLY helps adjusting values in the Propeller and ROS
        # when I can visualize the sensor output in RVIZ!
        # For this purpose, a lot of the parameters are a matter of personal taste.
        #   Whatever makes it easiest to visualize is best.
        # 2. I want to allow the cost map to use this data to avoid obstacles that the Kinect/Xtion miss.
        #     For the second purpose, some of the parameters here may need to be tweaked,
        #   to adjust how large an object looks to the cost map.
        # Note that we should also adjust the distance at which the cost map takes this data
        # into account either here or elsewhere.

        # Transform: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29

        # We do not need to broadcast a transform,
        # because it is static and contained within the URDF files now.
        # Here is the static transform for reference anyway:
        # self._SonarTransformBroadcaster.sendTransform(
        #     (0.1, 0.0, 0.2),
        #     (0, 0, 0, 1),
        #     ros_now,
        #     "sonar_laser",
        #     "base_link"
        #     )

        # Some help:
        # http://goo.gl/ZU9XrJ

        # I'm doing this all in degrees and then converting to Radians later.
        # Is there any way to do this in Radians?
        # I just don't know how to create and fill an array with "Radians"
        # since they are not rational numbers, but multiples of PI, thus the degrees.
        num_readings = 360  # How about 1 per degree?
        # num_reading_multiple = 2 # We have to track this so we know where to put the readings!
        # num_readings = 360 * num_reading_multiple
        laser_frequency = 100  # I'm not sure how to decide what to use here.
        # This is the fake distance to set all empty slots, and slots we consider "out of range"
        artificial_far_distance = 10
        # ranges = [1] * num_readings # Fill array with fake "1" readings for testing
        # Fill array with artificial_far_distance (not 0) and then overlap with real readings
        ping_ranges = [artificial_far_distance] * num_readings
        # If we use 0, then it won't clear the obstacles when we rotate away,
        # because costmap2d ignores 0's and Out of Range!
        # Fill array with artificial_far_distance (not 0) and then overlap with real readings
        ir_ranges = [artificial_far_distance] * num_readings

        # New idea here:
        # First, I do not think that this can be used for reliable for map generation.
        # If your room has objects that the Kinect
        # cannot map, then you will probably need to modify the room (cover mirrors, etc.) or try
        # other laser scanner options.
        # SO, since we only want to use it for cost planning, we should modify the data, because
        # it is easy for it to get bogged down with a lot of "stuff" everywhere.

        # From:
        # http://answers.ros.org/question/11446/costmaps-obstacle-does-not-clear-properly-under-sparse-environment/
        # "When clearing obstacles, costmap_2d only trusts laser scans returning a definite range.
        # Indoors, that makes sense. Outdoors, most scans return max range, which does not clear
        # intervening obstacles. A fake scan with slightly shorter ranges can be constructed that
        # does clear them out."
        # SO, we need to set all "hits" above the distance we want to pay attention to to a distance very far away,
        # but just within the range_max (which we can set to anything we want),
        # otherwise costmap will not clear items!
        # Also, 0 does not clear anything! So if we rotate, then it gets 0 at that point, and ignores it,
        # so we need to fill the unused slots with long distances.
        # NOTE: This does cause a "circle" to be drawn around the robot at the "artificialFarDistance",
        # but it shouldn't be a problem because we set
        # artificial_far_distance to a distance greater than the planner uses.
        # So while it clears things, it shouldn't cause a problem, and the Kinect should override it for things
        # in between.

        # Note that sensor orientation is important here!
        # If you have a different number or aim them differently this will not work!
        # The offset between the pretend sensor location in the URDF
        # and real location needs to be added to these values. This may need to be tweaked.
        sensor_offset = 0.217  # Measured, Calculated: 0.22545
        # This will be the max used range, anything beyond this is set to "artificial_far_distance"

        # Maximum distance accepted from the PING sensors.
        # Anything longer than this is assumed to be "no obstacle"
        # If we accept things too far away, we get too much noise.
        # 0.5 is a good default. Sometimes it is useful to extend it for testing
        # in RVIZ
        max_range_accepted = node.get_parameter("maxPingRangeAccepted").get_parameter_value().double_value

        # max_range_accepted Testing:
        # I think it is a trade-off, so there is no end to the adjustment that could be done.
        # I did a lot of testing with gmapping while building a map.
        # Obviously this would be slightly different from using a map we do not update.
        # It seems there are so many variables here that testing is difficult.
        # We could find one number works great in one situation but is hopeless in another.
        # Having a comprehensive test course to test in multiple modes for every possible value would be great,
        # but I think it would take months! :)
        # REMEMBER, the costmap only pays attention out to a certain set
        # for obstacle_range in costmap_common_params.yaml anyway.
        # Here are my notes for different values of "max_range_accepted":
        # 1 - looks good, and works ok, but
        # I am afraid that the costmap gets confused with things popping in and out of sight all of the time,
        # causing undue wandering.
        # 2 - This producing less wandering due to things popping in and out of the field of view,
        # BUT it also shows that we get odd affects at longer distances. i.e.
        #     A door frame almost always has a hit right in the middle of it.
        #     In a hallway there is often a hit in the middle about 1.5 meters out.
        # .5 - This works very well to have the PING data ONLY provide obstacle avoidance,
        # and immediately forget about said obstacles.
        #     This prevents the navigation stack from fighting with the Activity Board code's
        # built in safety stops, and instead navigate around obstacles before the Activity Board
        # code gets involved (other than to set speed reductions).
        #     The only down side is if you tell ArloBot to go somewhere that he cannot due to low obstacles,
        # he will try forever. He won't just bounce off of the obstacle,
        #     but he will keep trying it and then go away, turn around,
        # and try again over and over. He may even start wandering around
        # the facility trying to find another way in,
        #     but he will eventually come back and try it again.
        #     I'm not sure what the solution to this is though,
        # because avoiding low lying obstacles and adding low lying
        # features to the map are really two different things.
        #     I think if this is well tuned to avoid low lying obstacles it
        # probably will not work well for mapping features.
        #     IF we could map features with the PING sensors, we wouldn't need the 3D sensor. :)

        # NOTE: The above testing was done:
        #   1. Without a spread of PING sensors on the back.
        #   2. With a ASUS Xtion sensor that has a very narrow field of view in front, rather than
        #       the current RPLIDAR that has a 360 degree view of the room at all times.
        #   3. I don't even use the PING/IR for obstacle avoidance anymore. The ONLY reason they are
        #       coded here is to view them in Rviz.
        #      Instead they only affect the robot from inside the Activity board. This means the
        #       robot may repeatedly try to enter areas with low obstacles, but typically the PING
        #       sensors end up just preventing it from taking tight corners, and the ultimate
        #       solution is to provide a room that the robot can navigate with the "Lidar".

        #     NOTE: The bump sensors on Turtlebot mark but do not clear.
        # I'm not sure how that works out. It seems like every bump would
        # end up being a "blot" in the landscape never to be returned to,
        # but maybe there is something I am missing?

        # NOTE: Could this be different for PING vs. IR?
        # Currently I'm not using IR! Just PING. NEITHER are  being used by costmap.
        # It is here for seeing in RVIZ, and the Propeller board uses it for emergency stopping,
        # but costmap isn't watching any of this at the moment.
        #   Adding PING back in *COULD* be done, but I'm not sure it is wise.
        #   NEVER give IR to costmap, they are too erratic. If you need better obstacle detection
        #       than PING alone, consider the new LaserPING in place of the IR sensors.

        # Fill sensory arrays with sensor data
        start = 16
        end = start + self.serialInterface.pingCount
        telemetry_pingData = data[start:end]
        start = end
        end = start + self.serialInterface.irCount
        telemetry_irData = data[start:end]
        start = end
        end = start + self.serialInterface.floorSensorCount
        # telemetry_floorSensorData = data[start:end]
        start = end
        end = start + self.serialInterface.buttonCount
        telemetry_buttonInputData = data[start:end]
        start = end
        end = start + self.serialInterface.ledCount
        self._ledInputData_from_propeller = data[start:end]

        ping = [artificial_far_distance] * 10
        ir = [artificial_far_distance] * len(ping)

        # Convert cm to meters and add offset
        for i in range(0, len(ping)):
            # ping[0] = (int(line_parts[7]) / 100.0) + sensor_offset
            if len(telemetry_pingData) > i:
                ping[i] = telemetry_pingData[i] / 100.0 + sensor_offset
            else:
                ping[i] = artificial_far_distance
            # Set to "out of range" for distances over "max_range_accepted" to clear long range obstacles
            # and use this for near range only.
            if ping[i] > max_range_accepted:
                # Be sure "ultrasonic_scan.range_max" is set higher than this or
                # costmap will ignore these and not clear the cost map!
                ping[i] = artificial_far_distance
            if len(telemetry_irData) > i:
                ir[i] = telemetry_irData[i] / 100.0 + sensor_offset
            else:
                ir[i] = artificial_far_distance

        # Overwrite main sensors with upper deck sensors if they exist and are closer,
        # the positions are hard coded. :(

        if len(telemetry_pingData) > 10:
            upperSensor = telemetry_pingData[10] / 100.0 + sensor_offset
            if upperSensor < max_range_accepted and upperSensor < ping[1]:
                ping[1] = upperSensor
        if len(telemetry_pingData) > 11:
            upperSensor = telemetry_pingData[11] / 100.0 + sensor_offset
            if upperSensor < max_range_accepted and upperSensor < ping[2]:
                ping[2] = upperSensor
        if len(telemetry_pingData) > 12:
            upperSensor = telemetry_pingData[12] / 100.0 + sensor_offset
            if upperSensor < max_range_accepted and upperSensor < ping[3]:
                ping[3] = upperSensor
        if len(telemetry_pingData) > 13:
            upperSensor = telemetry_pingData[13] / 100.0 + sensor_offset
            if upperSensor < max_range_accepted and upperSensor < ping[7]:
                ping[7] = upperSensor

        # The sensors are 11cm from center to center at the front of the base plate.
        # The radius of the base plate is 22.545 cm
        # = 28 degree difference (http://ostermiller.org/calc/triangle.html)

        sensor_separation = 28

        # Fake Laser data from pings so that Mapping code  can use it as obstacles

        # for x in range(180 - sensor_spread / 2, 180 + sensor_spread / 2):
        ping_ranges[180 + sensor_separation * 2] = ping[5]
        ir_ranges[180 + sensor_separation * 2] = ir[5]

        ping_ranges[180 + sensor_separation] = ping[6]
        ir_ranges[180 + sensor_separation] = ir[6]

        ping_ranges[180] = ping[7]  # Rear Sensor
        ir_ranges[180] = ir[7]  # Rear Sensor

        ping_ranges[180 - sensor_separation] = ping[8]
        ir_ranges[180 - sensor_separation] = ir[8]

        ping_ranges[180 - sensor_separation * 2] = ping[9]
        ir_ranges[180 - sensor_separation * 2] = ir[9]

        # for x in range((360 - sensor_separation * 2) - sensor_spread / 2,
        #                (360 - sensor_separation * 2) + sensor_spread / 2):
        ping_ranges[360 - sensor_separation * 2] = ping[4]
        ir_ranges[360 - sensor_separation * 2] = ir[4]

        # for x in range((360 - sensor_separation) - sensor_spread / 2,
        #                (360 - sensor_separation) + sensor_spread / 2):
        ping_ranges[360 - sensor_separation] = ping[3]
        ir_ranges[360 - sensor_separation] = ir[3]

        # for x in range(360 - sensor_spread / 2, 360):
        # PINGranges[x] = ping[2]
        # IRranges[x] = ir[2]
        # Crosses center line
        # for x in range(0, sensor_spread /2):
        ping_ranges[0] = ping[2]
        ir_ranges[0] = ir[2]

        # for x in range(sensor_separation - sensor_spread / 2, sensor_separation + sensor_spread / 2):
        ping_ranges[sensor_separation] = ping[1]
        ir_ranges[sensor_separation] = ir[1]

        # for x in range((sensor_separation * 2) - sensor_spread / 2, (sensor_separation * 2) + sensor_spread / 2):
        ping_ranges[sensor_separation * 2] = ping[0]
        ir_ranges[sensor_separation * 2] = ir[0]

        # LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        ultrasonic_scan = LaserScan()
        infrared_scan = LaserScan()
        ultrasonic_scan.header.stamp = infrared_scan.header.stamp = now
        ultrasonic_scan.header.frame_id = "ping_sensor_array"
        infrared_scan.header.frame_id = "ir_sensor_array"
        # For example:
        # scan.angle_min = -45 * M_PI / 180; // -45 degree
        # scan.angle_max = 45 * M_PI / 180;   // 45 degree
        # if you want to receive a full 360 degrees scan,
        # you should try setting min_angle to -pi/2 and max_angle to 3/2 * pi.
        # Radians: http://en.wikipedia.org/wiki/Radian#Advantages_of_measuring_in_radians
        ultrasonic_scan.angle_min = infrared_scan.angle_min = 0.0
        # ultrasonic_scan.angle_max = 2 * 3.14159 # Full circle # Letting it use default, which I think is the same.
        # infrared_scan.angle_max = 2 * 3.14159 # Full circle # Letting it use default, which I think is the same.
        # ultrasonic_scan.scan_time = 3 # I think this is only really applied for 3D scanning
        # infrared_scan.scan_time = 3 # I think this is only really applied for 3D scanning
        # Make sure the part you divide by num_readings is the same as your angle_max!
        # Might even make sense to use a variable here?
        ultrasonic_scan.angle_increment = infrared_scan.angle_increment = (
                                                                                  2 * 3.14
                                                                          ) / num_readings
        # I'm not sure what the purpose of the time_increment is, but as of
        # but it seems to work fine if I supply NO time_increment
        # ultrasonic_scan.time_increment = infrared_scan.time_increment = (
        #     1 / laser_frequency
        # ) / num_readings
        # From: http://www.parallax.com/product/28015
        # Range: approximately 1 inch to 10 feet (2 cm to 3 m)
        # This should be adjusted based on the imaginary distance between the actual laser
        # and the laser location in the URDF file.
        # in Meters Distances below this number will be ignored REMEMBER the offset!
        ultrasonic_scan.range_min = infrared_scan.range_min = 0.02
        # in Meters Distances below this number will be ignored REMEMBER the offset!
        # This has to be above our "artificial_far_distance",
        # otherwise "hits" at artificial_far_distance will be ignored,
        # which means they will not be used to clear the cost map!
        # in Meters Distances above this will be ignored
        ultrasonic_scan.range_max = infrared_scan.range_max = (
                                                                      artificial_far_distance + 1
                                                              ) * 1.0
        ultrasonic_scan.ranges = ping_ranges
        infrared_scan.ranges = ir_ranges
        # "intensity" is a value specific to each laser scanner model.
        # It can safely be ignored

        self._UltraSonicPublisher.publish(ultrasonic_scan)
        self._InfraredPublisher.publish(infrared_scan)

        # Publish button pushes
        for entry in telemetry_buttonInputData:
            if entry == 1:
                node.get_logger().info("Button " + str(entry) + " was pushed.")
                arlo_buttons = ArloButtons()
                arlo_buttons.button_number = entry
                arlo_buttons.button_pressed = True
                self._buttons_publisher.publish(arlo_buttons)

    def start(self):
        self._OdomStationaryBroadcaster.Start()
        self.startSerialPort()

    def startSerialPort(self):
        self.serialInterface.Start()
        node.get_logger().info("Serial Data Gateway started by propeller_node.")
        self._serialAvailable = True

    def stop(self):
        """
        Called by ROS on shutdown.
        Shut off motors, record position and reset serial port.
        """
        node.get_logger().info("Stopping")
        self._SafeToOperate = False  # Prevent threads fighting
        # Save last position in parameter server in case we come up again without restarting roscore!
        # TODO: This won't work as ROS is going down, instead update the local environment variables.
        # node.set_parameters(
        #     [
        #         rclpy.parameter.Parameter('lastX', rclpy.Parameter.Type.DOUBLE, self.lastX),
        #         rclpy.parameter.Parameter('lastY', rclpy.Parameter.Type.DOUBLE, self.lastY),
        #         rclpy.parameter.Parameter('lastHeading', rclpy.Parameter.Type.DOUBLE, self.lastHeading),
        #     ]
        # )
        self._serialAvailable = False
        node.get_logger().info("Serial Interface stopping . . .")
        self.serialInterface.Stop()
        node.get_logger().info("Serial Interface stopped.")
        self._OdomStationaryBroadcaster.Stop()

    def _handle_velocity_command(self, twist_command):  # This is Propeller specific
        """ Handle movement requests. """
        # NOTE: turtlebot_node has a lot of code under its cmd_vel function
        # to deal with maximum and minimum speeds,
        # which are dealt with in ArloBot on the Propeller Board itself in the c code.
        if self._clear_to_go("forGeneralUse"):
            moveData = self.dataTypes.MoveDataPacket(
                twist_command.linear.x, twist_command.angular.z
            )
            self.serialInterface.SendToPropellerOverSerial("move", moveData, False)
            # For debugging
            # node.get_logger().warn(
            #     "SENT MOVE: "
            #     + str(twist_command.linear.x)
            #     + " "
            #     + str(twist_command.linear.y)
            # )
        elif self._clear_to_go("to_stop"):
            moveData = self.dataTypes.MoveDataPacket(0.0, 0.0)
            self.serialInterface.SendToPropellerOverSerial("move", moveData, False)

    def _toggleLED(self, LED):
        self.forwardTextToRosLog("_toggleLED")
        # Test with:
        # rosservice call /arlobot/ToggleLED 0 True
        # Or for all 5:
        # for i in 0 1 2 3 4;do rosservice call /arlobot/ToggleLED $i True;done

        # Note that we SET the DESIRED state in the list,
        # then it will get updated to Propeller if it isn't the same when the watchdog runs,
        # and then we'll get the new status via odometry
        # The return value is the CURRENT known value from the propeller though, not the one we set.
        # So if we don't trust THIS to get the job done, we can also basically say,
        # "Set True until Returns True

        # Lengthen array if it is called before it is filled
        while len(self._ledRequestedState_from_ROS) <= LED.led:
            self._ledRequestedState_from_ROS.append(0)

        self._ledRequestedState_from_ROS[LED.led] = 1 if LED.state else 0

        return self._ledInputData_from_propeller[LED.led]

    def _propellerReadyResponseFunction(self, data):
        self.forwardTextToRosLog("Propeller Activity Board Ready")
        # When the Propeller Board first boots it will send a 'ready' message
        # until it gets init data.
        node.get_logger().debug(data)

        # The Ready data includes a version number.
        # Now we should compare it to the version number in the code
        # on this computer to ensure that any updates made here
        # were sent to the Propeller board before this code was run.
        # If not, fail and display an error.
        propellerVersionNumber = checkPropellerCodeVersionNumber()
        if not self.serialInterface.propellerCodeVersion == propellerVersionNumber:
            node.get_logger().fatal("ERROR: Propeller Code does not match ROS Code!!!")
            node.get_logger().fatal(
                "Please use SimpleIDE to install the latest Propeller code to your Propeller Board: https://ekpyroticfrood.net/?p=165"
            )
            killScriptName = (
                    os.path.dirname(os.path.realpath(__file__))
                    + "/../../../../scripts/kill_ros.sh"
            )
            subprocess.call([killScriptName])

        node.get_logger().debug("Initialising Propeller Board.")
        initData = self.dataTypes.InitDataPacket(
            self.lastX, self.lastY, self.lastHeading
        )
        self.serialInterface.SendToPropellerOverSerial("init", initData, True)

        # NOTE You MUST also send Settings data after an INIT!
        # You CAN set the variable flagging to send settings, but what if
        # it sends ONE and the Propeller misses it?
        # The most sure fire way is to invalidate the config data we have,
        # forcing it to send until it is reset.
        self._config_from_propeller["trackWidth"] = 0.403
        self._config_from_propeller["distancePerCount"] = 0.00676
        self._config_from_propeller["wheelSymmetryError"] = 1.0
        # Set to opposites to  of self._settings_from_ros
        # to ensure we keep sending the settings
        # until we get the data from the board.
        self._config_from_propeller["ignoreProximity"] = not self._settings_from_ros[
            "ignoreProximity"
        ]
        self._config_from_propeller["ignoreCliffSensors"] = not self._settings_from_ros[
            "ignoreCliffSensors"
        ]
        self._config_from_propeller["ignoreIRSensors"] = not self._settings_from_ros[
            "ignoreIRSensors"
        ]
        self._config_from_propeller["ignoreFloorSensors"] = not self._settings_from_ros[
            "ignoreFloorSensors"
        ]
        self._config_from_propeller["pluggedIn"] = not self._settings_from_ros[
            "pluggedIn"
        ]

        self._getRosSettingsCompareAndSendToProp()

    def _propellerConfigDataHandler(self, data):
        node.get_logger().info("Propeller Config Data Received. Will read and compare to ROS data.")
        if len(data) > 7:  # Ignore short packets
            # Round these to the same precision as the input was given at
            # noinspection PyTypeChecker
            self._config_from_propeller["trackWidth"] = round(
                data[0], len(str(self._settings_from_ros["trackWidth"]).split(".")[1])
            )
            # noinspection PyTypeChecker
            self._config_from_propeller["distancePerCount"] = round(
                data[1],
                len(str(self._settings_from_ros["distancePerCount"]).split(".")[1]),
            )
            # noinspection PyTypeChecker
            self._config_from_propeller["wheelSymmetryError"] = round(
                data[2],
                len(str(self._settings_from_ros["wheelSymmetryError"]).split(".")[1]),
            )
            self._config_from_propeller["ignoreProximity"] = (
                True if data[3] == 1 else False
            )
            self._config_from_propeller["ignoreCliffSensors"] = (
                True if data[4] == 1 else False
            )
            self._config_from_propeller["ignoreIRSensors"] = (
                True if data[5] == 1 else False
            )
            self._config_from_propeller["ignoreFloorSensors"] = (
                True if data[6] == 1 else False
            )
            self._config_from_propeller["pluggedIn"] = True if data[7] == 1 else False

        self._getRosSettingsCompareAndSendToProp()

    def _broadcast_odometry(self, x, y, theta, vx, omega, now):

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(theta / 2.0)
        quaternion.w = cos(theta / 2.0)

        # First, we'll publish the transform from frame odom to frame base_link over tf
        # Note that sendTransform requires that 'to' is passed in before 'from' while
        # the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
        # This transform conflicts with transforms built into the Turtle stack
        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        # This is done in/with the robot_pose_ekf because it can integrate IMU/gyro data
        # using an "extended Kalman filter"
        # REMOVE this "line" if you use robot_pose_ekf

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = quaternion.x
        t.transform.rotation.y = quaternion.y
        t.transform.rotation.z = quaternion.z
        t.transform.rotation.w = quaternion.w

        self._OdometryTransformBroadcaster.sendTransform(t)

        # next, we will publish the odometry message over ROS
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = now
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation = quaternion

        odometry.child_frame_id = "base_link"
        odometry.twist.twist.linear.x = vx * 1.0
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.angular.z = omega * 1.0

        self._OdometryPublisher.publish(odometry)

    def _broadcast_static_odometry_info(self):
        """
        Broadcast last known odometry and transform while propeller board is offline
        so that ROS can continue to track status
        Otherwise things like mapping will fail when we loose our transform and publishing topics
        """

        if (
                self._broadcast_static_odometry
        ):  # Use motor status to decide when to broadcast static odometry:
            x = self.lastX
            y = self.lastY
            theta = self.lastHeading
            # If the Propeller Board is not updating the odometry we will assume the robot is still.
            vx = 0
            omega = 0
            self._broadcast_odometry(x, y, theta, vx, omega, node.get_clock().now().to_msg())

    def _getRosSettingsCompareAndSendToProp(self):
        self._updateSettingsFromROS()

        # self.forwardTextToRosLog(str(self._config_from_propeller))

        # Check if any settings need to be updated
        # noinspection Duplicates
        if (
                self._config_from_propeller["trackWidth"]
                != self._settings_from_ros["trackWidth"]
                or self._config_from_propeller["distancePerCount"]
                != self._settings_from_ros["distancePerCount"]
                or self._config_from_propeller["wheelSymmetryError"]
                != self._settings_from_ros["wheelSymmetryError"]
                or self._config_from_propeller["ignoreProximity"]
                != self._settings_from_ros["ignoreProximity"]
                or self._config_from_propeller["ignoreCliffSensors"]
                != self._settings_from_ros["ignoreCliffSensors"]
                or self._config_from_propeller["ignoreIRSensors"]
                != self._settings_from_ros["ignoreIRSensors"]
                or self._config_from_propeller["ignoreFloorSensors"]
                != self._settings_from_ros["ignoreFloorSensors"]
                or self._config_from_propeller["pluggedIn"]
                != self._settings_from_ros["pluggedIn"]
        ):
            node.get_logger().info("Sending settings from ROS to Propeller due to mismatch")
            settingsData = self.dataTypes.SettingsDataPacket(
                self._settings_from_ros["trackWidth"],
                self._settings_from_ros["distancePerCount"],
                self._settings_from_ros["wheelSymmetryError"],
                1 if self._settings_from_ros["ignoreProximity"] else 0,
                1 if self._settings_from_ros["ignoreCliffSensors"] else 0,
                1 if self._settings_from_ros["ignoreIRSensors"] else 0,
                1 if self._settings_from_ros["ignoreFloorSensors"] else 0,
                1 if self._settings_from_ros["pluggedIn"] else 0,
            )
            self.serialInterface.SendToPropellerOverSerial(
                "settings", settingsData, True
            )

    def watchDog(self):
        while rclpy.ok():
            # self.forwardTextToRosLog("watchDog")

            self._odometry_broadcast_timeout += 1
            if self._odometry_broadcast_timeout > self._odometry_broadcast_timeout_max:
                self._broadcast_static_odometry = True

            self._getRosSettingsCompareAndSendToProp()

            # self.r.sleep()
            # TODO: This is NOT the right way to do this, but it works for now.
            time.sleep(1)

    # Consolidate "clear to go" requirements here.
    def _clear_to_go(self, forWhat):
        # Required for all operations
        if (
                self._serialAvailable
                and self._SafeToOperate
                and self._safeToGo
                and self._leftMotorPower
                and self._rightMotorPower
        ):
            return True
        node.get_logger().info("Not clear to go:")
        node.get_logger().info(
            "_serialAvailable: " + str(self._serialAvailable) + ' ' +
            "_SafeToOperate: " + str(self._SafeToOperate) + ' ' +
            "_safeToGo: " + str(self._safeToGo) + ' ' +
            "_leftMotorPower: " + str(self._leftMotorPower) + ' ' +
            "_rightMotorPower: " + str(self._rightMotorPower) + ' '
        )
        return False


def main(args=None):
    # rclpy.init(args=args)
    propellerComm = PropellerComm()
    # rclpy.on_shutdown(propellerComm.stop)
    try:
        propellerComm.start()
        node.get_logger().info("Propellerbot_node has started.")
        # TODO Putting this watchDog here prevents spin from ever running.
        # propellerComm.watchDog()
        # TODO: Without spin running, subscribers don't work.
        rclpy.spin(node)

        # TODO: Perhaps we can move all code OUT of the watchdog to other places?

        # TODO: After that this may be able to be refactored a bit to better replicate the example ROS2 nodes
        # TODO: In these `self` seems to BE node and the node is initialized in main

        # TODO: it is possible to stop propeller node but not ROS, not sure how that affects odometry?

        # TODO: A stop command is NOT sent upon shutdown (although propeller board stops runaway after its own timeout)

    except KeyboardInterrupt:
        propellerComm.stop()


if __name__ == '__main__':
    main()
