#!/usr/bin/env python
import rospy
import subprocess
import os
import fnmatch
from std_msgs.msg import Bool
from arlobot_msgs.msg import arloSafety
from arlobot_msgs.srv import UnPlug

'''
This node will monitor various items and let ROS know if it is safe
for the robot to move or not.
The main item is whether the laptop is plugged into power or not.

This node will also allow other nodes to request that the robot
shut down or stop.
This should be usable from web interfaces, or possibly other sensors.
'''

class ArlobotSafety(object):

    def __init__(self):
        rospy.init_node('arlobot_safety')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        self.r = rospy.Rate(1) # 1hz refresh rate

        # Global variable for whether we've been asked to unplug or not
        self._unPlug = False

        # Track battery
        self._laptopBatteryPercent = 100

        # I am going to set the AC power status as a parameter, so that it can be checked by low priority nodes,
        # and publish the "arloSafety" as a topic so that it can be subscribed to and acted upon immediately
        self.acPower = True # Status of whether laptop is plugged in or not. We assume 1, connected, to start with because that is the most restrictive state.
        rospy.set_param('~ACpower', self.acPower) # Publish initial state

        self._safetyStatusPublisher = rospy.Publisher('~safetyStatus', arloSafety, queue_size=1) # for publishing status of AC adapter

        unplugger = rospy.Service('arlobot_unplug', UnPlug, self._handle_unplug_request)

    def Stop(self):
        rospy.loginfo("ArlobotSafety id is shutting down.")
        # Delete plugged in status, since it is no longer a valid parameter without anyone to monitor it
        if rospy.has_param('~ACpower'):
            rospy.delete_param('~ACpower')

    def Run(self):
        while not rospy.is_shutdown():
            #rospy.loginfo("Looping . . .")

            # Check computer's AC power status and set it as a ROS parameter
            if rospy.has_param('/arlobot/monitorACconnection'): # If arlobot_bringup is running
                checkAC = rospy.get_param('/arlobot/monitorACconnection') # Use parameter from arlobot_bringup to decide if we should monitor AC or not
            else:
                checkAC = True # Otherwise monitor it if arlobot_bringup isn't running

            if checkAC: # Unless we were told not to
                #upower -i /org/freedesktop/UPower/devices/line_power_AC
                laptopPowerState = subprocess.Popen(['upower', '-d', '/org/freedesktop/UPower/devices/line_power_AC'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True)
                # Only grab the FIRST battery percentage!
                gotBatteryPercent = False
                for line in iter(laptopPowerState.stdout.readline, ""):
                    #rospy.loginfo(line) # for Debugging
                    if 'online' in line:
                        #rospy.loginfo(line) # Just so we know it is working (for debugging)
                        upowerOutput = line.split()
                        #print upowerOutput[1]
                        if upowerOutput[1] == 'no':
                            if self.acPower: # Only log and set parameters if there is a change!
                                rospy.loginfo("AC Power DISconnected.")
                                self.acPower = False
                                rospy.set_param('~ACpower', self.acPower)
                        elif upowerOutput[1] == 'yes':
                            if self.acPower is False: # Only log and set parameters if there is a change!
                                rospy.loginfo("AC Power Connected.")
                                self.acPower = True
                                rospy.set_param('~ACpower', self.acPower)
                    if 'percentage' in line and not gotBatteryPercent:
                        upowerOutput = line.split()
                        #rospy.loginfo(upowerOutput[1].rstrip('%'))
                        self._laptopBatteryPercent = int(upowerOutput[1].rstrip('%'))
                        gotBatteryPercent = True
                laptopPowerState.stdout.close()
                laptopPowerState.wait()
            else: # Just set to 0 if we were told to ignore AC power status.
                if self.acPower: # Only log and set parameters if there is a change!
                    self.acPower = False
                    rospy.set_param('~ACpower', self.acPower)

            # arloSafty Status message
            safety_status = arloSafety()

            safety_status.laptopBatteryPercent = self._laptopBatteryPercent

            # Set AC Power status message
            safety_status.acPower = self.acPower

            # Determine safety status based on what we know
            if self.acPower:
                safety_status.safeToGo = False
            else:
                safety_status.safeToGo = True
                if self._unPlug:
                    # Turn off unPlug now that it is unplugged
                    self._unPlug = False

            safety_status.unPlugging = self._unPlug
            if safety_status.unPlugging:
                # If we've been asked to unplug, then set safeToGo
                safety_status.safeToGo = True

            # This is a new status meant to separate being safe to MOVE,
            # from being safe to OPERATE, so that the motors and Activity Board
            # do not shut down every time we need to be still,
            # and instead we can just be still
            safety_status.safeToOperate = True
            # For now it is just tagged "True", but if we find
            # a reason to do a full shutdown, use this!

            # Check for external "STOP" calls:
            # Any external calls will override everything else!
            # This allows any program anywhere to put a file named
            # STOP into ~/.arlobot/status and stop the robot.
            # This folder should exist, otherwise we are going to stop!
            status_dir = os.path.expanduser("~/.arlobot/status")
            if os.path.isdir(status_dir):
                for file in os.listdir(status_dir):
                    if fnmatch.fnmatch(file, 'STOP'):
                        safety_status.safeToGo = False
            else:
                safety_status.safeToGo = False

            # This checks for files left by the door checking system.
            # Note that if we are exploring you will have to just
            # ANY file will stop the robot.
            # The web server has a button to delete all files in the
            # ~/.arlobot/status/doors folder.
            door_dir = os.path.expanduser("~/.arlobot/status/doors")
            # This folder should exist, otherwise we are going to stop!
            if os.path.isdir(door_dir):
                map_name = rospy.get_param("/arlobot/mapname", "empty")
                # If we do not know the name of a map
                if map_name == 'empty':
                    for file in os.listdir(door_dir):
                        # Then ANY file in this folder will stop the robot
                        if fnmatch.fnmatch(file, '*'):
                            safety_status.safeToGo = False
                else:
                    for file in os.listdir(door_dir):
                        # If we know the map name, only look for files with
                        # the map_name as a prefix so that other doors
                        # do not stop us.
                        if fnmatch.fnmatch(file, map_name+'-*'):
                            safety_status.safeToGo = False
            else:
                safety_status.safeToGo = False

            self._safetyStatusPublisher.publish(safety_status) # Publish safety status

            self.r.sleep() # Sleep long enough to maintain the rate set in __init__

    def _handle_unplug_request(self, request):
        if request:
            rospy.loginfo("Unplug requested.")
        else:
            rospy.loginfo("Unplug cancel requested.")
        self._unPlug = request.unPlug
        return True

if __name__ == '__main__':
    node = ArlobotSafety()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
