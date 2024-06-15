#!/usr/bin/env python
import rospy
import subprocess
import os
import fnmatch
from arlobot_ros.msg import ArloSafety
from arlobot_ros.srv import UnPlug

"""
This node will monitor various items and let ROS know if it is safe
for the robot to move or not.
The main item is whether the laptop is plugged into power or not.

This node will also allow other nodes to request that the robot
shut down or stop.
This should be usable from web interfaces, or possibly other sensors.
"""


class ArlobotSafety(object):
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node("arlobot_safety")
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        self.r = rospy.Rate(1)  # 1hz refresh rate

        # Global variable for whether we've been asked to unplug or not
        self._unPlug = False

        # Track battery
        self._laptopBatteryPercent = 100

        # I am going to set the AC power status as a parameter, so that it can be checked by low priority nodes,
        # and publish the "ArloSafety" as a topic so that it can be subscribed to and acted upon immediately
        self._acPower = True  # Status of whether laptop is plugged in or not. We assume 1, connected, to start with because that is the most restrictive state.
        rospy.set_param("~ACpower", self._acPower)  # Publish initial state

        self._safetyStatusPublisher = node.create_publisher(ArloSafety, queue_size=1
        , 
            "~safetyStatus")  # for publishing status of AC adapter

        node.create_service(UnPlug, "arlobot_unplug", self._handle_unplug_request)

    def Stop(self):
        rospy.loginfo("ArlobotSafety id is shutting down.")
        # Delete plugged in status, since it is no longer a valid parameter without anyone to monitor it
        if rospy.has_param("~ACpower"):
            rospy.delete_param("~ACpower")

    def Run(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("Looping . . .")

            # Check computer's AC power status and set it as a ROS parameter
            if rospy.has_param(
                "/arlobot/monitorACconnection"
            ):  # If arlobot_ros is running
                checkAC = rospy.get_param(
                    "/arlobot/monitorACconnection"
                )  # Use parameter from arlobot_ros to decide if we should monitor AC or not
            else:
                checkAC = True  # Otherwise monitor it if arlobot_ros isn't running

            if checkAC:  # Unless we were told not to
                # upower -i /org/freedesktop/UPower/devices/line_power_AC
                laptopPowerState = subprocess.run(
                    ["upower", "-d", "/org/freedesktop/UPower/devices/line_power_AC"],
                    stdout=subprocess.PIPE,
                ).stdout.decode("utf-8")
                # Only grab the FIRST battery percentage!
                gotBatteryPercent = False
                for line in laptopPowerState.split("\n"):
                    if "online" in line:
                        upowerOutput = line.split()
                        if upowerOutput[1] == "no":
                            if (
                                self._acPower
                            ):  # Only log and set parameters if there is a change!
                                rospy.loginfo("AC Power DISconnected.")
                                self._acPower = False
                                rospy.set_param("~ACpower", self._acPower)
                        elif upowerOutput[1] == "yes":
                            if (
                                self._acPower is False
                            ):  # Only log and set parameters if there is a change!
                                rospy.loginfo("AC Power Connected.")
                                self._acPower = True
                                rospy.set_param("~ACpower", self._acPower)
                    if "percentage" in line and not gotBatteryPercent:
                        upowerOutput = line.split()
                        self._laptopBatteryPercent = int(upowerOutput[1].rstrip("%"))
                        gotBatteryPercent = True
            else:  # Just set to 0 if we were told to ignore AC power status.
                if self._acPower:  # Only log and set parameters if there is a change!
                    self._acPower = False
                    rospy.set_param("~ACpower", self._acPower)

            # ArloSafety Status message
            safety_status = ArloSafety()

            safety_status.laptop_battery_percent = self._laptopBatteryPercent

            # Set AC Power status message
            safety_status.ac_power = self._acPower

            # Determine safety status based on what we know
            if self._acPower:
                safety_status.safe_to_go = False
            else:
                safety_status.safe_to_go = True
                if self._unPlug:
                    # Turn off unPlug now that it is unplugged
                    self._unPlug = False

            safety_status.unplugging = self._unPlug
            if safety_status.unplugging:
                # If we've been asked to unplug, then set safe_to_go to True to allow movement
                safety_status.safe_to_go = True

            # This is a new status meant to separate being safe to MOVE,
            # from being safe to OPERATE, so that the motors and Activity Board
            # do not shut down every time we need to be still,
            # and instead we can just be still
            safety_status.safe_to_operate = True
            # For now it is just tagged "True", but if we find
            # a reason to do a full shutdown, use this!

            # Check for external "STOP" calls:
            # Any external calls will override everything else!
            # This allows any program anywhere to put a file named
            # STOP into ~/.arlobot/status and stop the robot.
            # This folder should exist, otherwise we are going to stop!
            status_dir = os.path.expanduser("~/.arlobot/status")
            if os.path.isdir(status_dir):
                for file_name in os.listdir(status_dir):
                    if fnmatch.fnmatch(file_name, "STOP"):
                        safety_status.safe_to_go = False
            else:
                safety_status.safe_to_go = False

            # Check for open doors
            if rospy.has_param("/arlobot/monitorDoors"):  # If arlobot_ros is running
                monitorDoors = rospy.get_param(
                    "/arlobot/monitorDoors"
                )  # Use parameter from arlobot_ros to decide if we should monitor AC or not
            else:
                monitorDoors = False  # Default is false
            if monitorDoors:
                doorStatus = subprocess.run(
                    [
                        "node",
                        os.path.dirname(os.path.realpath(__file__))
                        + "/../../node/doorClosed.js",
                    ],
                    stdout=subprocess.PIPE,
                ).stdout.decode("utf-8")
                dangerousDoorsOpen = doorStatus != "true\n"
                safety_status.dangerous_doors_open = dangerousDoorsOpen
                if dangerousDoorsOpen:
                    safety_status.safe_to_go = False

            self._safetyStatusPublisher.publish(safety_status)  # Publish safety status

            self.r.sleep()  # Sleep long enough to maintain the rate set in __init__

    def _handle_unplug_request(self, request):
        if request:
            rospy.loginfo("Unplug requested.")
        else:
            rospy.loginfo("Unplug cancel requested.")
        self._unPlug = request.unplug
        return True


if __name__ == "__main__":
    node = ArlobotSafety()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
