#!/usr/bin/env python
# Using Black: https://github.com/ambv/black
# pylint: disable=line-too-long
# Software License Agreement (BSD License)
#
# Author: Chris L8 https://github.com/chrisl8
# URL: https://github.com/chrisl8/ArloBot
#
# Derived from \opt\ros\hydro\lib\create_node\turtlebot_node.py
# This is based on turtlebot_node adapted to run on a Propeller Activity Board based ArloBot
#
# Special thanks to arduino.py by Dr. Rainer Hessmer
# https://code.google.com/p/drh-robotics-ros/
#
# NOTE: This script requires parameters to be loaded from ~/.arlobot/arlobot.yaml!

from __future__ import print_function
import rospy
import tf2_ros
from math import sin, cos
import time
import numpy.ma as ma
import os
import subprocess

from geometry_msgs.msg import Quaternion, TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from arlobot_msgs.msg import usbRelayStatus, arloStatus, arloSafety, arloButtons
from arlobot_msgs.srv import FindRelay, ToggleRelay, ToggleLED

from checkPropellerCodeVersionNumber import checkPropellerCodeVersionNumber
from PropellerSerialInterface import PropellerSerialInterface
from PropellerSerialDataPacketTypes import PropellerSerialDataPacketTypes
from OdomStationaryBroadcaster import OdomStationaryBroadcaster


class PropellerComm(object):
    """
    Helper class for communicating with a Propeller board over serial port
    """

    def __init__(self):
        rospy.init_node("arlobot")

        self.r = rospy.Rate(1)  # 1hz refresh rate
        self._motorsOn = (
            False
        )  # Set to True if the motors are on, used with USB Relay Control board
        self._safeToGo = False  # Use arlobot_safety to set this
        self._SafeToOperate = False  # Use arlobot_safety to set this
        self._unPlugging = False  # Used for when arlobot_safety tells us to "UnPlug"!
        self._wasUnplugging = (
            False
        )  # Track previous unplugging status for motor control
        self._SwitchingMotors = False  # Prevent overlapping calls to _switch_motors
        self._serialAvailable = False
        self._odometry_broadcast_timeout = 0
        self._odometry_broadcast_timeout_max = 2
        self._broadcast_static_odometry = False
        self._leftMotorPower = False
        self._rightMotorPower = False
        self._laptop_battery_percent = 100
        # Store last x, y and heading for reuse when we reset
        # I took off the ~, because that was causing these to reset to default on every restart
        # even if roscore was still up.
        self.lastX = rospy.get_param("lastX", 0.0)
        self.lastY = rospy.get_param("lastY", 0.0)
        self.lastHeading = rospy.get_param("lastHeading", 0.0)
        self.alternate_heading = self.lastHeading

        # Get motor relay numbers for use later in _HandleUSBRelayStatus if USB Relay is in use:
        self.relayExists = rospy.get_param("~usbRelayInstalled", False)
        if self.relayExists:
            # I think it is better to get these once than on every run of _HandleUSBRelayStatus
            self.usbLeftMotorRelayLabel = rospy.get_param("~usbLeftMotorRelayLabel", "")
            self.usbRightMotorRelayLabel = rospy.get_param(
                "~usbRightMotorRelayLabel", ""
            )
            rospy.loginfo("Waiting for USB Relay find_relay service to start . . .")
            rospy.wait_for_service("/arlobot_usbrelay/find_relay")
            rospy.loginfo("USB Relay find_relay service started.")
            try:
                find_relay = rospy.ServiceProxy(
                    "/arlobot_usbrelay/find_relay", FindRelay
                )
                self.leftMotorRelay = find_relay(self.usbLeftMotorRelayLabel)
                self.rightMotorRelay = find_relay(self.usbRightMotorRelayLabel)
                if self.leftMotorRelay.foundRelay and self.leftMotorRelay.foundRelay:
                    rospy.loginfo(
                        "Left = "
                        + str(self.leftMotorRelay.relayNumber)
                        + " & Right = "
                        + str(self.rightMotorRelay.relayNumber)
                    )
                else:
                    self.relayExists = False
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % e)
            rospy.Subscriber(
                "arlobot_usbrelay/usbRelayStatus",
                usbRelayStatus,
                self._handle_usb_relay_status,
            )  # Safety Shutdown

        # Store the data from ROS
        self._settings_from_ros = {
            "trackWidth": 0.0,
            "distancePerCount": 0.0,
            "wheelSymmetryError": 0.0,
            "ignoreProximity": False,
            "ignoreCliffSensors": False,
            "ignoreIRSensors": False,
            "ignoreFloorSensors": False,
            "pluggedIn": True,
        }

        self._updateSettingsFromROS()

        # Store the incoming data from the Propeller board
        self._config_from_propeller = {
            "trackWidth": 0,
            "distancePerCount": 0,
            "wheelSymmetryError": 0,
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
        rospy.Subscriber("cmd_vel", Twist, self._handle_velocity_command, queue_size=1)
        # NOTE: Keep the cmd_vel subscription queue_size to 1, because we never want to allow a backlog of twist commands. Always just send the most recent one.
        # Otherwise strange behavior occurs, even the robot responding to commands very late, like stopping and then starting a command given moments ago!
        rospy.Subscriber(
            "arlobot_safety/safetyStatus",
            arloSafety,
            self._safety_shutdown,
            queue_size=1,
        )  # Safety Shutdown

        # Publishers
        # for publishing PIR status
        # self._pirPublisher = rospy.Publisher("~pirState", Bool, queue_size=1)
        self._arlo_status_publisher = rospy.Publisher(
            "arlo_status", arloStatus, queue_size=1
        )
        self._buttons_publisher = rospy.Publisher("buttons", arloButtons, queue_size=1)

        # IF the Odometry Transform is done with the robot_pose_ekf do not publish it,
        # but we are not using robot_pose_ekf, because it does nothing for us if you don't have a full IMU!
        # REMOVE this line if you use robot_pose_ekf
        self._OdometryTransformBroadcaster = tf2_ros.TransformBroadcaster()

        self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=1)

        # We don't need to broadcast a transform, as it is static and contained within the URDF files
        # self._SonarTransformBroadcaster = tf2_ros.TransformBroadcaster()
        self._UltraSonicPublisher = rospy.Publisher(
            "ultrasonic_scan", LaserScan, queue_size=1
        )
        self._InfraredPublisher = rospy.Publisher(
            "infrared_scan", LaserScan, queue_size=1
        )

        # For Camera Joint Testing
        # self._JointStatePublisher = rospy.Publisher(
        #     "joint_states", JointState, queue_size=1
        # )

        # Create a service that can be called to send robot to a map based goal
        # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

        rospy.Service("~ToggleLED", ToggleLED, self._toggleLED)

        # You can use the ~/catkin_ws/src/ArloBot/scripts/find_ActivityBoard.sh script to find this, and
        # You can set it by running this before starting this:
        # rosparam set /arlobot/port $(~/catkin_ws/src/ArloBot/scripts/find_ActivityBoard.sh)
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud_rate = int(rospy.get_param("~baudRate", 115200))

        rospy.loginfo(
            "Starting with serial port: " + port + ", baud rate: " + str(baud_rate)
        )

        self.serialInterface = PropellerSerialInterface(
            self._propellerReadyResponseFunction,
            self._propellerOdomDataHandler,
            self._propellerConfigDataHandler,
            self.TestDataResponseFunction,
            self.displaySerialErrors,
            port,
            baud_rate,
        )
        self.dataTypes = PropellerSerialDataPacketTypes()

        self._OdomStationaryBroadcaster = OdomStationaryBroadcaster(
            self._broadcast_static_odometry_info
        )

    def displaySerialErrors(self, data):
        data = str(data)
        if "error" in data.lower():
            rospy.logwarn(data)
        elif "Serial Write Delay:" in data:
            rospy.logdebug(data)
        elif "Good Packet Delay:" in data:
            rospy.logdebug(data)
        else:
            rospy.loginfo(data)

    def TestDataResponseFunction(self, data):
        rospy.loginfo("Test Packet Received: " + str(data))

    def _updateSettingsFromROS(self):
        self._settings_from_ros["trackWidth"] = rospy.get_param(
            "~driveGeometry/trackWidth", "0"
        )
        self._settings_from_ros["distancePerCount"] = rospy.get_param(
            "~driveGeometry/distancePerCount", "0"
        )
        self._settings_from_ros["wheelSymmetryError"] = rospy.get_param(
            "~driveGeometry/wheelSymmetryError", "0"
        )
        self._settings_from_ros["ignoreProximity"] = rospy.get_param(
            "~ignoreProximity", False
        )
        self._settings_from_ros["ignoreCliffSensors"] = rospy.get_param(
            "~ignoreCliffSensors", False
        )
        self._settings_from_ros["ignoreIRSensors"] = rospy.get_param(
            "~ignoreIRSensors", False
        )
        self._settings_from_ros["ignoreFloorSensors"] = rospy.get_param(
            "~ignoreFloorSensors", False
        )

    def _handle_usb_relay_status(self, status):
        """
        Update motor status based on real data from arlobot_usbrelay topic
        """
        # This should never get called otherwise, but just in case the relay is in use but the motors are not.
        if self.relayExists:
            # print (status.relayOn)
            # print (status.relayOn[self.leftMotorRelay.relayNumber])
            # print (status.relayOn[self.leftMotorRelay.relayNumber])
            # Zero indexed arrays!
            if (
                status.relayOn[self.leftMotorRelay.relayNumber - 1]
                and status.relayOn[self.rightMotorRelay.relayNumber - 1]
            ):
                self._motorsOn = True
            else:
                self._motorsOn = False

    def _safety_shutdown(self, status):
        """
        Prevent sending twist commands if the SafeToOperate topic goes false.
        Set unPlugging variable to allow for safe unplug operation.
        """
        self._unPlugging = status.unPlugging
        self._SafeToOperate = status.safeToOperate
        self._safeToGo = status.safeToGo

        self._settings_from_ros["pluggedIn"] = status.acPower

        self._laptop_battery_percent = status.laptopBatteryPercent

    # noinspection Duplicates
    def _propellerOdomDataHandler(self, data):
        """
        Broadcast all data from propeller monitored sensors on the appropriate topics.
        """

        now = rospy.Time.now()

        self._odometry_broadcast_timeout = 0
        self._broadcast_static_odometry = False

        # If we got this far, we can assume that the Propeller board is initialized and the motors should be on.
        # The _switch_motors() function will deal with the _SafeToOperate issue
        if not self._motorsOn:
            self._switch_motors(True)

        # Check for short lines, though this should never happen, as the serial receiver checks this.
        if (
            len(data) < 17
        ):  # Just discard short/long lines, increment this as lines get longer
            rospy.logwarn("Short odometry line from Propeller board: " + str(len(data)))
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
        # rospy.logwarn(
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
        # rospy.logwarn("vx: " + str(vx) + " omega: " + str(omega))
        # if self.lastHeading == theta and abs(vx) > 0:
        #     rospy.logwarn("No movement recorded but vx = " + str(vx))
        #     rospy.logwarn(
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
        arlo_status = arloStatus()
        # Order from ROS Interface for ArloBot.c
        left_motor_voltage = (15 / 4.69) * data[6]
        right_motor_voltage = (15 / 4.69) * data[7]
        arlo_status.robotBatteryLevel = 12.0
        if left_motor_voltage < 1:
            arlo_status.leftMotorPower = False
        else:
            arlo_status.leftMotorPower = True
            arlo_status.robotBatteryLevel = left_motor_voltage
        self._leftMotorPower = arlo_status.leftMotorPower
        if right_motor_voltage < 1:
            arlo_status.rightMotorPower = False
        else:
            arlo_status.rightMotorPower = True
            arlo_status.robotBatteryLevel = right_motor_voltage
        self._rightMotorPower = arlo_status.rightMotorPower
        # 11.6 volts is the cutoff for an SLA battery.
        if arlo_status.robotBatteryLevel < 12:
            arlo_status.robotBatteryLow = True
        else:
            arlo_status.robotBatteryLow = False

        arlo_status.abd_speedLimit = data[8]
        arlo_status.abdR_speedLimit = data[9]
        arlo_status.Heading = self.lastHeading
        arlo_status.gyroHeading = self.alternate_heading
        arlo_status.minDistanceSensor = data[10]
        arlo_status.safeToProceed = True if data[11] == 1 else False
        arlo_status.safeToRecede = True if data[12] == 1 else False
        arlo_status.Escaping = True if data[13] == 1 else False
        arlo_status.cliff = True if data[14] == 1 else False
        arlo_status.floorObstacle = True if data[15] == 1 else False

        arlo_status.laptopBatteryPercent = self._laptop_battery_percent
        arlo_status.acPower = self._settings_from_ros["pluggedIn"]
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
        # js.header.stamp = rospy.Time.now()
        # js.header.frame_id = "camera_rgb_joint"
        # js.name = ["camera_rgb_joint"]
        # js.position = [3]
        # self._JointStatePublisher.publish(js)

        # Fake laser from "PING" Ultrasonic Sensor and IR Distance Sensor input:
        # http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
        # Use:
        # roslaunch arlobot_rviz_launchers view_robot.launch
        # to view this well for debugging and testing.

        # The purpose of this is two fold:
        # 1. It REALLY helps adjusting values in the Propeller and ROS
        # when I can visualize the sensor output in RVIZ!
        # For this purpose, a lot of the parameters are a matter of personal taste.
        #   Whatever makes it easiest to visualize is best.
        # 2. I want to allow AMCL to use this data to avoid obstacles that the Kinect/Xtion miss.
        #     For the second purpose, some of the parameters here may need to be tweaked,
        #   to adjust how large an object looks to AMCL.
        # Note that we should also adjust the distance at which AMCL takes this data
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

        # Use:
        # roslaunch arlobot_rviz_launchers view_robot.launch
        # to view this well for debugging and testing.

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
        max_range_accepted = rospy.get_param("/arlobot/maxPingRangeAccepted", 0.5)

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
        # Right now when the robot spins, it clears the obstacles behind it,
        # because there are fewer sensors on the back side.
        # If the obstacle was seen all of the way around the robot, in the same spot,
        # it may stop returning to the same location as soon as it turns around?

        #     NOTE: The bump sensors on Turtlebot mark but do not clear.
        # I'm not sure how that works out. It seems like every bump would
        # end up being a "blot" in the landscape never to be returned to,
        # but maybe there is something I am missing?

        # NOTE: Could this be different for PING vs. IR?
        # Currently I'm not using IR! Just PING. The IR is not being used by costmap.
        # It is here for seeing in RVIZ, and the Propeller board uses it for emergency stopping,
        # but costmap isn't watching it at the moment. I think it is too erratic for that.

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
        ultrasonic_scan.header.stamp = now
        infrared_scan.header.stamp = now
        ultrasonic_scan.header.frame_id = "ping_sensor_array"
        infrared_scan.header.frame_id = "ir_sensor_array"
        # For example:
        # scan.angle_min = -45 * M_PI / 180; // -45 degree
        # scan.angle_max = 45 * M_PI / 180;   // 45 degree
        # if you want to receive a full 360 degrees scan,
        # you should try setting min_angle to -pi/2 and max_angle to 3/2 * pi.
        # Radians: http://en.wikipedia.org/wiki/Radian#Advantages_of_measuring_in_radians
        ultrasonic_scan.angle_min = 0
        infrared_scan.angle_min = 0
        # ultrasonic_scan.angle_max = 2 * 3.14159 # Full circle # Letting it use default, which I think is the same.
        # infrared_scan.angle_max = 2 * 3.14159 # Full circle # Letting it use default, which I think is the same.
        # ultrasonic_scan.scan_time = 3 # I think this is only really applied for 3D scanning
        # infrared_scan.scan_time = 3 # I think this is only really applied for 3D scanning
        # Make sure the part you divide by num_readings is the same as your angle_max!
        # Might even make sense to use a variable here?
        ultrasonic_scan.angle_increment = (2 * 3.14) / num_readings
        infrared_scan.angle_increment = (2 * 3.14) / num_readings
        ultrasonic_scan.time_increment = (1 / laser_frequency) / num_readings
        infrared_scan.time_increment = (1 / laser_frequency) / num_readings
        # From: http://www.parallax.com/product/28015
        # Range: approximately 1 inch to 10 feet (2 cm to 3 m)
        # This should be adjusted based on the imaginary distance between the actual laser
        # and the laser location in the URDF file.
        # in Meters Distances below this number will be ignored REMEMBER the offset!
        ultrasonic_scan.range_min = 0.02
        # in Meters Distances below this number will be ignored REMEMBER the offset!
        infrared_scan.range_min = 0.02
        # This has to be above our "artificial_far_distance",
        # otherwise "hits" at artificial_far_distance will be ignored,
        # which means they will not be used to clear the cost map!
        # in Meters Distances above this will be ignored
        ultrasonic_scan.range_max = artificial_far_distance + 1
        # in Meters Distances above this will be ignored
        infrared_scan.range_max = artificial_far_distance + 1
        ultrasonic_scan.ranges = ping_ranges
        infrared_scan.ranges = ir_ranges
        # "intensity" is a value specific to each laser scanner model.
        # It can safely be ignored

        self._UltraSonicPublisher.publish(ultrasonic_scan)
        self._InfraredPublisher.publish(infrared_scan)

        # Publish button pushes
        for entry in telemetry_buttonInputData:
            if entry == 1:
                rospy.loginfo("Button " + str(entry) + " was pushed.")
                arlo_buttons = arloButtons()
                arlo_buttons.buttonNumber = entry
                arlo_buttons.buttonPressed = True
                self._buttons_publisher.publish(arlo_buttons)

    def start(self):
        self._OdomStationaryBroadcaster.Start()
        self.startSerialPort()

    def startSerialPort(self):
        self.serialInterface.Start()
        rospy.loginfo("Serial Data Gateway started by propellerbot_node.")
        self._serialAvailable = True

    def stop(self):
        """
        Called by ROS on shutdown.
        Shut off motors, record position and reset serial port.
        """
        rospy.loginfo("Stopping")
        self._SafeToOperate = False  # Prevent threads fighting
        # Save last position in parameter server in case we come up again without restarting roscore!
        rospy.set_param("lastX", self.lastX)
        rospy.set_param("lastY", self.lastY)
        rospy.set_param("lastHeading", self.lastHeading)
        if self.relayExists:
            time.sleep(5)  # Give the motors time to shut off
        self._serialAvailable = False
        rospy.loginfo("Serial Interface stopping . . .")
        self.serialInterface.Stop()
        rospy.loginfo("Serial Interface stopped.")
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
            self.serialInterface.SendToPropellerOverSerial("move", moveData)
            # For debugging
            # rospy.logwarn(
            #     "SENT MOVE: "
            #     + str(twist_command.linear.x)
            #     + " "
            #     + str(twist_command.linear.y)
            # )
        elif self._clear_to_go("to_stop"):
            moveData = self.dataTypes.MoveDataPacket(0.0, 0.0)
            self.serialInterface.SendToPropellerOverSerial("move", moveData)

    def _toggleLED(self, LED):
        # Test with:
        # rosservice call /arlobot/ToggleLED 0 True
        # Or for all 5:
        # for i in 0 1 2 3 4;do rosservice call /arlobot/ToggleLED $i True;done

        # Note that we SET the DESIRED state in the list,
        # then it will get updated to Propeller if it isn't the same when the watchdog runs,
        # and then we'll get the new status via odometry
        # The return value is the CURRENT known value from the propeller though, not the one we set.
        # So if we don't trust THIS to get the job done, we an also basically say,
        # "Set True until Returns True

        # Lengthen array if it is called before it is filled
        while len(self._ledRequestedState_from_ROS) <= LED.led:
            self._ledRequestedState_from_ROS.append(0)

        self._ledRequestedState_from_ROS[LED.led] = 1 if LED.state else 0

        return self._ledInputData_from_propeller[LED.led]

    # TODO: Implement all functions that can be sent to/from Propeller,
    # TODO: Including Test
    # TODO: and put these together in the code here.
    # TODO: Test all Propeller data types

    def _propellerReadyResponseFunction(self, data):
        # When the Propeller Board first boots it will send a 'ready' message
        # until it gets init data.
        rospy.logdebug(data)

        # The Ready data includes a version number.
        # Now we should compare it to the version number in the code
        # on this computer to ensure that any updates made here
        # were sent to the Propeller board before this code was run.
        # If not, fail and display an error.
        propellerVersionNumber = checkPropellerCodeVersionNumber()
        if not self.serialInterface.propellerCodeVersion == propellerVersionNumber:
            rospy.logfatal("ERROR: Propeller Code does not match ROS Code!!!")
            rospy.logfatal(
                "Please use SimpleIDE to install the latest Propeller code to your Propeller Board: https://ekpyroticfrood.net/?p=165"
            )
            killScriptName = (
                os.path.dirname(os.path.realpath(__file__))
                + "/../../../../scripts/kill_ros.sh"
            )
            subprocess.call([killScriptName])

        rospy.logdebug("Initialising Propeller Board.")
        initData = self.dataTypes.InitDataPacket(
            self.lastX, self.lastY, self.lastHeading
        )
        self.serialInterface.SendToPropellerOverSerial("init", initData, False)

        # NOTE You MUST also send Settings data after an INIT!
        # You CAN set the variable flagging to send settings, but what if
        # it sends ONE and the Propeller misses it?
        # The most sure fire way is to invalidate the config data we have,
        # forcing it to send until it is reset.
        self._config_from_propeller["trackWidth"] = 0
        self._config_from_propeller["distancePerCount"] = 0
        self._config_from_propeller["wheelSymmetryError"] = 0
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

        # TODO: These are no longer in service. Consider reimplementing somehow:
        # self._pirPublisher.publish(True)
        # self._pirPublisher.publish(False)

    def _propellerConfigDataHandler(self, data):
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
        odometry.pose.pose.position.z = 0
        odometry.pose.pose.orientation = quaternion

        odometry.child_frame_id = "base_link"
        odometry.twist.twist.linear.x = vx
        odometry.twist.twist.linear.y = 0
        odometry.twist.twist.angular.z = omega

        self._OdometryPublisher.publish(odometry)

    def _broadcast_static_odometry_info(self):
        """
        Broadcast last known odometry and transform while propeller board is offline
        so that ROS can continue to track status
        Otherwise things like gmapping will fail when we loose our transform and publishing topics
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
            self._broadcast_odometry(x, y, theta, vx, omega, rospy.Time.now())

    def _switch_motors(self, state):
        """ Switch Motors on and off as needed. """
        # Relay control was moved to its own package
        if self.relayExists:
            if not self._SwitchingMotors:  # Prevent overlapping runs
                self._SwitchingMotors = True
                # Switch "on" to "off" if not safe to operate,
                # then we can just pass state to arlobot_usbrelay
                if not self._SafeToOperate:
                    state = False
                rospy.wait_for_service("/arlobot_usbrelay/toggle_relay")
                rospy.loginfo("Switching motors.")
                try:
                    toggle_relay = rospy.ServiceProxy(
                        "/arlobot_usbrelay/toggle_relay", ToggleRelay
                    )
                    left_relay_result = toggle_relay(self.usbLeftMotorRelayLabel, state)
                    right_relay_result = toggle_relay(
                        self.usbRightMotorRelayLabel, state
                    )
                    if (
                        left_relay_result.toggleSuccess
                        and right_relay_result.toggleSuccess
                    ):
                        self._motorsOn = True
                    else:
                        self._motorsOn = False
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: %s" % e)
                self._SwitchingMotors = False
        else:  # If no automated motor control exists, just set the state blindly.
            self._motorsOn = state

    def watchDog(self):
        while not rospy.is_shutdown():

            self._odometry_broadcast_timeout += 1
            if self._odometry_broadcast_timeout > self._odometry_broadcast_timeout_max:
                self._broadcast_static_odometry = True

            if self._unPlugging or self._wasUnplugging:
                self.UnplugRobot()

            self._updateSettingsFromROS()

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
                    "settings", settingsData, False
                )

            # Check if any LEDs need to be updated
            for index, setting in enumerate(self._ledRequestedState_from_ROS):
                if self._ledInputData_from_propeller[index] != setting:
                    if self._serialAvailable:
                        ledData = self.dataTypes.LEDDataPacket(index, setting)
                        self.serialInterface.SendToPropellerOverSerial("led", ledData)

            self.r.sleep()

    def UnplugRobot(self):
        if (
            self._unPlugging
            and not self._wasUnplugging
            and self._clear_to_go("forUnplugging")
        ):
            # We will only do this once, and let it continue until AC power is disconnected
            self._wasUnplugging = True
            # Slow backup until unplugged
            # This should be a slow backward crawl
            # Minimum Linear Velocity: 0.06 m/s (18 TPS)
            rospy.loginfo("Unplugging!")
            moveData = self.dataTypes.MoveDataPacket(-0.06, 0.0)
            self.serialInterface.SendToPropellerOverSerial("move", moveData)
        # Once we are unplugged, stop the robot before returning control to handle_velocity_command
        # And we only need permission to stop at this point.
        if (
            self._wasUnplugging
            and not self._settings_from_ros["pluggedIn"]
            and self._serialAvailable
        ):
            rospy.loginfo("Unplugging complete")
            self._wasUnplugging = False
            moveData = self.dataTypes.MoveDataPacket(0.0, 0.0)
            self.serialInterface.SendToPropellerOverSerial("move", moveData)
        # Finally, if we were unplugging, but something went wrong, we should stop the robot
        # Since no one else will do this while we have the "_wasUnplugging" variable
        # set.
        if (
            self._wasUnplugging
            and not self._clear_to_go("forUnplugging")
            and self._serialAvailable
        ):
            self._wasUnplugging = False
            moveData = self.dataTypes.MoveDataPacket(0.0, 0.0)
            self.serialInterface.SendToPropellerOverSerial("move", moveData)

    # Consolidate "clear to go" requirements here.
    def _clear_to_go(self, forWhat):
        return_value = False
        # Required for all operations
        if (
            self._serialAvailable
            and self._SafeToOperate
            and self._safeToGo
            and self._motorsOn
            and self._leftMotorPower
            and self._rightMotorPower
        ):
            return_value = True
        # Negations by use case
        if forWhat == "forUnplugging":
            # Unplugging should only happen if AC is connected
            if not self._settings_from_ros["pluggedIn"]:
                return_value = False
        if forWhat == "forGeneralUse":
            # The handle_velocity_command should only operate if the robot is unplugged,
            # and the unplugging function of the Watchdog process is not in control
            if self._settings_from_ros["pluggedIn"] or self._wasUnplugging:
                return_value = False
        # Special Cases
        if forWhat == "to_stop":
            # handle_velocity_command can send a STOP if it is unsafe to do anything else,
            # but the serial connection is up,
            # as long as the unplugging operation is not in progress
            # This should bring the robot to a halt in response to any other request,
            # if anything is amiss.
            if self._serialAvailable and not self._wasUnplugging:
                return_value = True
            else:
                return_value = False
        return return_value


if __name__ == "__main__":
    propellerComm = PropellerComm()
    rospy.on_shutdown(propellerComm.stop)
    try:
        propellerComm.start()
        rospy.loginfo("Propellerbot_node has started.")
        propellerComm.watchDog()

    except rospy.ROSInterruptException:
        propellerComm.stop()
