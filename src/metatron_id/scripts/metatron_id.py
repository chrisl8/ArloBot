#!/usr/bin/env python
import rospy

import subprocess
from metatron_services.srv import *
from metatron_services.msg import *
from os.path import expanduser


# This node is being/has been replaced by the Node.js code in node/

# This is the "id" for my personal instance of ArloBot named Metatron
# The purpose of this node is to provide Metatron with basic, internally motivated,
# goals, plans and functions.
# This node will launch and monitor ArloBot nodes which would otherwise be
# started by a user.
# This node will control the other nodes and packages via direct methods:
# i.e. Using subprocess.Popen to launch packages
# and indirect methods:
# i.e. Publishing messages over topics that other nodes will use.
#
# This node IS a ROS node, but it will likely operate more like a monolithic
# program than a ROS package. Nevertheless, I will endeavor to break down clear
# functions into other packages, I will place anything generic into ArloBot instead of here,
# and I will use ROS methods to communicate with other nodes.


class MetatronId(object):
    def __init__(self):
        rospy.init_node('metatron_id')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        self.r = rospy.Rate(1)  # 1hz refresh rate

        self.throttleCount = 0  # Use this to throttle things like screen shots
        self.throttleTop = 15  # Set how long to wait, raise to slow it down more.

        # ArloBot Minimal package status
        self.arloMinimal = 0  # Track whether the arlo_bringup minimal package has been brought up and is running.
        self.mapToUse = ""
        self._MetaTronStatusPublisher = rospy.Publisher('~status', metatron_status, queue_size=1)
        self.script_location = expanduser(rospy.get_param("/metatron_id/script_location", "~"))

        # Don't forget to add your services to CMakeLists.txt
        # QUESTION: Is there any point to this, or should this just be in the metatron launch file?
        arlo_minimal_toggle = rospy.Service('~toggle_arlo_minimal', ToggleArloMinimal, self._toggle_arlo_minimal)
        set_map = rospy.Service('~set_map', SetMap, self._set_map)

        # If you get an error like this:
        # ERROR: service [/metatron_id/toggle_camera] responded with an error: error processing request:
        # _ToggleCameras() takes exactly 3 arguments (2 given)
        # Remember that the arguments are sent as ONE unit, a list, not one argument per item.

    def stop(self):
        rospy.loginfo("Metatron id is shutting down.")
        try:
            self.arloProcess.terminate()
        except:
            rospy.loginfo("No ArloBot process exists to shut down.")
        else:
            rospy.loginfo("ArloBot ROS Package has been shut down.")

        try:
            self.cameraProcess.terminate()
        except:
            rospy.loginfo("No Camera process exists to shut down.")
        else:
            rospy.loginfo("Camera has been shut down.")

    def be(self):
        while not rospy.is_shutdown():
            # Message for sending with publisher:
            status_to_publish = metatron_status()

            # Activities that need to be throttled (not happen too often)
            self.throttleCount += 1
            if self.throttleCount > self.throttleTop:
                self.throttleCount = 0

            # Publish status of objects handled by this node:
            status_to_publish.arlobotMinimal = self.arloMinimal
            # statusToPublish.leftMotorOn = self.leftMotorOn
            # statusToPublish.rightMotorOn = self.rightMotorOn
            # statusToPublish.upperLightOn = self.upperLightOn
            # statusToPublish.lowerLightOn = self.lowerLightOn

            status_to_publish.mapToUse = self.mapToUse

            self._MetaTronStatusPublisher.publish(status_to_publish)

            self.r.sleep()  # Sleep long enough to maintain the rate set in __init__

    def _toggle_arlo_minimal(self, req):
        # This will be exposed as a service, so that it can be called from anything in ROS,
        # like a rosbridge enabled web page to start and stop the robot's basic services.
        if req.state:  # "True" is on, "False" is off, because sending "on" to a ROS service sends True, not text
            # rospy.set_param('arlo_bot/safeToPowerOn', True)
            # rospy.set_param('arlo_bot/pluggedIn', False)
            try:
                self.arloProcess.poll()
            except:
                rospy.loginfo("Starting ArloBot ROS Package.")
                self.arloProcess = subprocess.Popen(['roslaunch', 'arlobot_bringup', 'minimal.launch'], close_fds=True)
                self.arloMinimal = True
            else:
                if self.arloProcess.poll() == 0:
                    rospy.loginfo("Restarting ArloBot ROS Package that was previously shut down.")
                    self.arloProcess = subprocess.Popen(['roslaunch', 'arlobot_bringup', 'minimal.launch'],
                                                        close_fds=True)
                    self.arloMinimal = True
        else:  # Assume if it isn't True it is false/"off"
            # rospy.set_param('arlo_bot/safeToPowerOn', False)
            # rospy.set_param('arlo_bot/pluggedIn', True)
            try:
                self.arloProcess.poll()
            except:
                rospy.logdebug("No ArloBot process exists to shut down.")
            else:
                if self.arloProcess.poll() != 0:
                    self.arloProcess.terminate()
                    # Do not do this if you don't have stdout or stderr set to PIPE or it will crash!
                    # (could use try catch)
                    # self.arloProcess.stdout.close()
                    self.arloProcess.wait()
                    # self.arloProcess.close() # .close appears to be an incorrect thing to call on a subprocess
                    rospy.loginfo("ArloBot ROS Package has been shut down upon request.")
                    self.arloMinimal = False
                elif self.arloProcess.poll() == 0:
                    rospy.logdebug("ArloBot process has already been shut down.")
        return True

    def _set_map(self, req):
        self.mapToUse = req.map
        return True

if __name__ == '__main__':
    node = MetatronId()
    rospy.on_shutdown(node.stop)
    try:
        node.be()
    except rospy.ROSInterruptException:
        node.stop()
