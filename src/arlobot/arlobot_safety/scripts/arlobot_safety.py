#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool

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

        # I am going to set the AC power status as a parameter, so that it can be checked by low priority nodes,
        # and publish the "safeToGo" as a topic so that it can be subscribed to and acted upon immediately
        self.acPower = True # Status of whether laptop is plugged in or not. We assume 1, connected, to start with because that is the most restrictive state.
        rospy.set_param('~ACpower', self.acPower) # Publish initial state

        self.safeToGo = False # Set false as default until we check things
        self._safetyStatusPublisher = rospy.Publisher('~safeToGo', Bool, queue_size=1) # for publishing status of AC adapter

        
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
            
            # Determine safety status based on what we know
            if self.acPower:
                self.safeToGo = False
            else:
                self.safeToGo = True
            
            self._safetyStatusPublisher.publish(self.safeToGo) # Publish safety status
            self.r.sleep() # Sleep long enough to maintain the rate set in __init__
            
if __name__ == '__main__':
    node = ArlobotSafety()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
