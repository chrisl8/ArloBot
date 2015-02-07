#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool
from arlobot_msgs.msg import arloSafety
from arlobot_msgs.srv import UnPlug
from geometry_msgs.msg import Twist

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
                laptopPowerState = subprocess.Popen(['upower', '-i', '/org/freedesktop/UPower/devices/line_power_AC'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True)
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
                laptopPowerState.stdout.close()
                laptopPowerState.wait()
            else: # Just set to 0 if we were told to ignore AC power status.
                if self.acPower: # Only log and set parameters if there is a change!
                    self.acPower = False
                    rospy.set_param('~ACpower', self.acPower)
            
            # Set AC Power status message
            safety_status.acPower = self.acPower

            # arloSafty Status message
            safety_status = arloSafety()
            sendTwist = False

            # Determine safety status based on what we know
            if self.acPower:
                safety_status.safeToGo = False
            else:
                safety_status.safeToGo = True
                if self._unPlug:
                    # Turn off unPlug now that it is unplugged
                    self._unPlug = False
                    # Send one last twist message to stop the robot
                    sendTwist = True

            safety_status.unPlugging = self._unPlug
            if safety_status.unPlugging:
                # If we've been asked to unplug, then set safeToGo
                safety_status.safeToGo = True
                # Send Twist messages to make it move the robot
                sendTwist = True
            
            self._safetyStatusPublisher.publish(safety_status) # Publish safety status

            if sendTwist:
                # Send Twist command if robot is to be unplugging itself.
                # The actual value of the twist command will not be used.
                # In order to maintain a 100% safe unplug, the propellerbot_node
                # will ignore the value of the twist command while in "unPlugging" mode
                # and send its own value.
                # This allows the propellerbot_node's _handle_velocity_command
                # to stay in charge, while not requiring a specal "loop" within the node code
                # to accomplish the unpugging.
                pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
                twist = Twist()
                # This is a "dummy" twist command, so set to 0, in case it DOES get run.
                linear_speed = 0
                angular_speed = 0
                twist.linear.x = linear_speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_speed
                for x in range(0, 10):
                    pub.publish(twist)

            self.r.sleep() # Sleep long enough to maintain the rate set in __init__

    def _handle_unplug_request(self, request):
        rospy.loginfo("Unplug requested.")
        self._unPlug = request.unPlug
        return True

if __name__ == '__main__':
    node = ArlobotSafety()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
