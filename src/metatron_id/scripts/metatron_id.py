#!/usr/bin/env python
import rospy

import subprocess
from metatron_services.srv import *
from metatron_services.msg import *
from os.path import expanduser

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
#
# TODO: This may be a good node to experiment with ROS Java!
#
# NOTE: This is slowly becoming the web publisher node. :)


class MetatronId(object):
    def __init__(self):
        rospy.init_node('metatron_id')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        self.r = rospy.Rate(1)  # 1hz refresh rate

        self.throttleCount = 0  # Use this to throttle things like screen shots
        self.throttleTop = 15  # Set how long to wait, raise to slow it down more.

        # ArloBot Minimal package status
        self.arloMinimal = 0  # Track whether the arlo_bringup minimal package has been brought up and is running.
        self.cameraState = False  # Track camera state
        self.cameraChoice = ""  # Track camera choice
        self.mapToUse = ""
        self._MetaTronStatusPublisher = rospy.Publisher('~status', metatron_status, queue_size=1)
        self.script_location = expanduser(rospy.get_param("/metatron_id/script_location", "~"))
        self.web_folder = expanduser(rospy.get_param("/metatron_id/web_folder", "~"))

        # Cameras - This is very setup dependent!
        self.camera1 = rospy.get_param("/camera1", "/dev/video0")
        self.camera2 = rospy.get_param("/camera2", "/dev/video0")

        # Don't forget to add your services to CMakeLists.txt
        camera_toggle = rospy.Service('~toggle_camera', ToggleCamera, self._toggle_cameras)
        # QUESTION: Is there any point to this, or should this just be in the metatron launch file?
        arlo_minimal_toggle = rospy.Service('~toggle_arlo_minimal', ToggleArloMinimal, self._toggle_arlo_minimal)
        self.newScreenShot = False  # Global so _wake_screen can set it
        screen_wake = rospy.Service('~wake_screen', WakeScreen, self._wake_screen)

        set_map = rospy.Service('~set_map', SetMap, self._set_map)

        # If you get an error like this:
        #ERROR: service [/metatron_id/toggle_camera] responded with an error: error processing request:
        #_ToggleCameras() takes exactly 3 arguments (2 given)
        #Remember that the arguments are sent as ONE unit, a list, not one argument per item.

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
            status_to_publish.newScreenShot = self.newScreenShot  # Pick up true if wakeScreen set it
            self.newScreenShot = False  # Reset to false for default next round

            # Activities that need to be throttled (not happen too often)
            self.throttleCount += 1
            if self.throttleCount > self.throttleTop:
                self.throttleCount = 0
                # Save a screenshot of the X Desktop for use on web interface:
                #"DISPLAY=:0.0 import -window root ~/arloweb/xscreen.png"
                # http://www.thegeekstuff.com/2012/08/screenshot-ubuntu/
                process = subprocess.Popen(
                    "DISPLAY=:0.0 /usr/bin/import -window root " + self.web_folder + "/xscreen.png", shell=True)
                # Wait for it to finish, this should be instantaneous.
                process.wait()
                # This will tell the web interface to grab a new copy, avoiding waisted network traffic
                status_to_publish.newScreenShot = True
                # Taking a screenshot every second provides almost a real time display on the web site!
                # But it adds CPU load, and a bit of network bandwidth. So this is still way cool!

            # Publish status of objects handled by this node:
            status_to_publish.arlobotMinimal = self.arloMinimal
            if not self.cameraState:
                status_to_publish.camera1 = False
                status_to_publish.camera2 = False
            elif self.cameraChoice == "laptop":  # Laptop will be Camera 2, because it is less used.
                status_to_publish.camera1 = False
                status_to_publish.camera2 = True
            else:
                status_to_publish.camera1 = True
                status_to_publish.camera2 = False
            #statusToPublish.leftMotorOn = self.leftMotorOn
            #statusToPublish.rightMotorOn = self.rightMotorOn
            #statusToPublish.upperLightOn = self.upperLightOn
            #statusToPublish.lowerLightOn = self.lowerLightOn

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
                    #self.arloProcess.close() # .close appears to be an incorrect thing to call on a subprocess
                    rospy.loginfo("ArloBot ROS Package has been shut down upon request.")
                    self.arloMinimal = False
                elif self.arloProcess.poll() == 0:
                    rospy.logdebug("ArloBot process has already been shut down.")
        return True

    def _wake_screen(self, req):
        # Then take a fresh screen shot now rather than waiting for the next round
        process = subprocess.Popen("DISPLAY=:0.0 /usr/bin/import -window root " + self.web_folder + "/xscreen.png",
                                   shell=True)
        process.wait()  # Wait for it to finish, this should be instantaneous.
        # This will tell the web interface to grab a new copy, avoiding waisted network traffic
        self.newScreenShot = True
        return True

    def _set_map(self, req):
        self.mapToUse = req.map
        return True

    def _toggle_cameras(self, req):
        # This will be exposed as a service, so that it can be called from anything in ROS,
        # like a rosbridge enabled web page to start and stop the robot's basic services.

        self.cameraChoice = req.camera
        if req.camera == "laptop":
            video_command = ['mjpg_streamer', '-i', '/usr/local/lib/input_uvc.so -d ' + self.camera2 + ' -n -f 30 -r 1280x720',
                             '-o',
                             '/usr/local/lib/output_http.so -p 58180 -w ' + self.script_location + '/mjpg-streamer/mjpg-streamer/www']
        else:  # Just assume we want the primary camera if we get a bogus choice
            video_command = ['mjpg_streamer', '-i', '/usr/local/lib/input_uvc.so -d ' + self.camera1 + ' -n -f 30 -r 1280x720',
                             '-o',
                             '/usr/local/lib/output_http.so -p 58180 -w ' + self.script_location + '/mjpg-streamer/mjpg-streamer/www']

        if req.state:  # "True" is on, "False" is off, because sending "on" to a ROS service sends True, not text
            try:
                self.cameraProcess.poll()
            except:
                rospy.loginfo("Starting " + str(req.camera) + " camera.")
                self.cameraProcess = subprocess.Popen(video_command, shell=False)
                self.cameraState = True
            else:  # https://docs.python.org/2/library/subprocess.html#subprocess.Popen.returncode
                print self.cameraProcess.poll()
                try:
                    self.cameraProcess.terminate()
                    # Do not do this if you don't have stdout or stderr set to PIPE or it will crash!
                    # (could use try catch)
                    # self.cameraProcess.stdout.close()
                    self.cameraProcess.wait()
                    #self.cameraProcess.close() # .close appears to be an incorrect thing to call on a subprocess
                    rospy.loginfo("Camera ROS Package has been shut down upon request.")
                except:
                    print ("Camera terminate Failed")
                rospy.loginfo("Restarting Camera ROS Package that was previously shut down.")
                self.cameraProcess = subprocess.Popen(video_command, shell=False)
                self.cameraState = True
        else:  # Assume if it isn't True it is false/"off"
            try:
                self.cameraProcess.poll()
            except:
                rospy.logdebug("No Camera process exists to shut down.")
            else:
                try:
                    self.cameraProcess.terminate()
                    self.cameraProcess.wait()
                except:
                    print ("Camera terminate Failed")
                rospy.loginfo("Camera ROS Package has been shut down upon request.")
                self.cameraState = False
        return True


if __name__ == '__main__':
    node = MetatronId()
    rospy.on_shutdown(node.stop)
    try:
        node.be()
    except rospy.ROSInterruptException:
        node.stop()
