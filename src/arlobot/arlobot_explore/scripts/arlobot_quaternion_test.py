#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool
import move_base_msgs.msg
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from hector_nav_msgs.srv import GetRobotTrajectory # It says 'msgs' but it is a srv!
import tf

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalID

'''
An attempt at "autonomous" navigation.
See:
http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
'''

class ArlobotExplore(object):

    def __init__(self):
        rospy.init_node('arlobot_explore')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        #self.r = rospy.Rate(1) # 1hz refresh rate
        
        # Creates the SimpleActionClient, passing the type of the action
        #self._MoveBaseClient = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        
        # Global variable to hold current pose
        self.currentOdom = Odometry()
        
        # Subscribe to the current pose via odometry and populate our own variable with the data
        '''
        I'm not sure of any other way to do this. I'd like to just "grab" it at a point in time, but subscriptions don't work that way.
        '''
        rospy.Subscriber("odom", Odometry, self._SetCurrentOdom)
        
        # I am going to set the AC power status as a parameter, so that it can be checked by low priority nodes,
        # and publish the "safeToGo" as a topic so that it can be subscribed to and acted upon immediately
        #self.acPower = True # Status of whether laptop is plugged in or not. We assume 1, connected, to start with because that is the most restrictive state.
        #rospy.set_param('~ACpower', self.acPower) # Publish initial state

        #self.safeToGo = False # Set false as default until we check things
        #self._safetyStatusPublisher = rospy.Publisher('~safeToGo', Bool, queue_size=1) # for publishing status of AC adapter

    def _SetCurrentOdom(self, currentOdom):
        self.currentOdom = currentOdom
        
    def Stop(self):
        rospy.loginfo("ArlobotExplore is shutting down.")
        #self._MoveBaseClient.cancel_all_goals() # In case the poor thing is still stuck trying to go nowhere!
        
    def Run(self):
        # Waits until the action server has started up and started
        # listening for goals.
        '''
        This will stall until the move_base comes up,
        in other words, if you don't run gmapping before this, this will jsut wait,
        and it won't go on until gmapping says "odom received!"
        '''
        self._MoveBaseClient.wait_for_server()
        rospy.loginfo("move_base is UP!")
        
        goal = move_base_msgs.msg.MoveBaseGoal()
        #print("Empty goal:")
        #print(goal)
        # Note that move_base will not go to an all zero target.
        
        # Grab a static copy of the current pose to work with
        #Otherwise it might change under our feet!
        '''
        Note, the actual pose on the map is not the same as this,
        but there is not map based pose.
        What there is the odometry based pose, and then a transform
        from the odometry to the map.
        Retriving the transform, combining it with the odom pose
        and making use of it is a future exercise.
        '''
        current_odom = self.currentOdom
        #print("Current odom:")
        #print(current_odom)
        print("current_odom.pose:")
        print(current_odom.pose)
        #rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
        
        rosNow = rospy.Time.now()
        #we'll create a goal to send to move_base
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rosNow

        # This will move forward 1 meter from 0
        #goal.target_pose.pose.position.x = 0.0
        #goal.target_pose.pose.orientation.w = 1.0
        
        # Set new pose to same as current pose
        '''
        You have to set .position and .orientation,
        not .pose because the current_odom.pose
        includes covariance, the other cannot take
        '''
        goal.target_pose.pose.position = current_odom.pose.pose.position
        goal.target_pose.pose.orientation = current_odom.pose.pose.orientation
        '''
        If the odometry, which is tied to /base_link, was identical
        to the map location, this would tell it to go nowhere,
        but what we actually end up doing here is telling move_base
        to move the robot the difference between the odom (/base_link)
        and the map. :)
        '''
        '''
        a quick and easy way to get the transform from the /map to /base_link is to use the command-line tool:
        rosrun tf tf_echo /map /base_link
        So how do I combine this myself?
        '''
        
        # Rotate currentPose by 90 degrees
        quaternion_difference = tf.transformations.quaternion_about_axis(0.123, (1, 0, 0))
        #print("quaternion_difference:")
        #print(quaternion_difference)

        print("Populated goal:")
        print(goal.target_pose.pose)

        rospy.loginfo("Sending goal");
        # Sends the goal to the action server.
        self._MoveBaseClient.send_goal(goal)

        rospy.loginfo("Waiting for response . . .");
        # Waits for the server to finish performing the action.
        self._MoveBaseClient.wait_for_result()
        #This could wait a VERY long time,
        #if the move_base doesn't have a timeout it will never come back,
        #in most cases it does, but it seems in some cases it will retry forever.
        # http://docs.ros.org/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a460c9f52fd650f918cb287765f169445
        
        result = self._MoveBaseClient.get_result()
        #rospy.loginfo(result)
        result = self._MoveBaseClient.get_state()
        #rospy.loginfo(result)
        
        current_odom = self.currentOdom
        print("New odom:")
        print(current_odom.pose)

        rospy.loginfo("Ok, now what?")

if __name__ == '__main__':
    node = ArlobotExplore()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
