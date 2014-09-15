#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool
import move_base_msgs.msg
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from hector_nav_msgs.srv import GetRobotTrajectory # It says 'msgs' but it is a srv!
import tf
import math

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

#TODO: Test for robot movement before sending it places! It could get really goofy to send to -90 from it's position ten seconds ago!

    def __init__(self):
        rospy.init_node('arlobot_explore')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        #self.r = rospy.Rate(1) # 1hz refresh rate
        
        # Creates the SimpleActionClient, passing the type of the action
        self._MoveBaseClient = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        
        # Listen to the transforms http://wiki.ros.org/tf/TfUsingPython
        self.tf_listener = tf.listener.TransformListener()
        rospy.sleep(2) # If you call self.tf_listener too soon it has no data in the listener buffer!
        # http://answers.ros.org/question/164911/move_base-and-extrapolation-errors-into-the-future/
        
        # Global variable to hold current pose
        self.currentOdom = Odometry()
        
        # Subscribe to the current pose via odometry and populate our own variable with the data
        '''
        I'm not sure of any other way to do this. I'd like to just "grab" it at a point in time, but subscriptions don't work taht way.
        '''
        rospy.Subscriber("odom", Odometry, self._SetCurrentOdom)
        # Turns out this works great if you have no map and just want to make movements based on odometry,
        # but if you are using a map, you need the /map to /base_link transform!
        
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
        self._MoveBaseClient.cancel_all_goals() # In case the poor thing is still stuck trying to go nowhere!
        
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
        #print("current_odom.pose:")
        #print(current_odom.pose)
        #rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
        
        rosNow = rospy.Time.now()
        #we'll create a goal to send to move_base
        # If you are just sending commands to the robot with no map use base_link
        #goal.target_pose.header.frame_id = "base_link"
        # But if you have gmapping active and are using a map, you need to use the map!
        goal.target_pose.header.frame_id = "map"
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
        #goal.target_pose.pose.position = current_odom.pose.pose.position
        #goal.target_pose.pose.orientation = current_odom.pose.pose.orientation
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
        
        # Testing stay still
        '''
        If I get the CURRENT position and send it, it should stay still, right?
        
        chrisl8@ArloBot:~/arlobot$ rosrun tf tf_echo /map /base_link
        At time 1410717899.809
        - Translation: [1.777, 0.951, 0.101]
        - Rotation: in Quaternion [0.000, 0.000, -0.501, 0.865]
        
        This works IF you set the goal.target_pose.header.frame_id = "map"
        '''
        '''
        goal.target_pose.pose.position.x = 1.777
        goal.target_pose.pose.position.y = 0.951
        goal.target_pose.pose.position.z = 0.101
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = -0.501
        goal.target_pose.pose.orientation.w = 0.865
        '''
        # This works, it stays still!
        
        # Rotate by 90 degrees
        '''
        - Translation: [1.777, 0.951, 0.101]
        - Rotation: in Quaternion [0.000, 0.000, -0.501, 0.865]
        Rotated left about a quarter turn:
        - Translation: [1.781, 0.930, 0.101]
        - Rotation: in Quaternion [0.000, 0.000, 0.397, 0.918]

        '''
        #quaternion_about_axis(angle, axis):
        #origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        '''
        http://wiki.ros.org/geometry/CoordinateFrameConventions
        Units
        Distances are in meters
        Angles are in radians
        Orientation
        X forward
        Y left
        Z up
        '''
        #z is UP, so we rotate around taht.
        # 90 degrees = 1.57079633 radians
        quaternion_difference = tf.transformations.quaternion_about_axis(1.57079633, (0, 0, 1))
        #print("quaternion_difference:")
        #print(quaternion_difference)
        #[-0.         -0.         -0.70710678  0.70710678]
        # 0.707 is recognizable as a 90 degree turn in quaternions.
        new_quaternion = tf.transformations.quaternion_multiply([0.000, 0.000, -0.501, 0.865], quaternion_difference)
        #print(new_quaternion)
        #[0.000, 0.000, -0.422, 0.907]
        # Current location:
        #- Translation: [1.725, 1.027, 0.101]
        #- Rotation: in Quaternion [0.000, 0.000, -0.422, 0.907]

        goal.target_pose.pose.position.x = 1.725
        goal.target_pose.pose.position.y = 1.027
        goal.target_pose.pose.position.z = 0.101
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        # It works, we rotated left 90 degrees!
        
        # Rotate by -90 degrees assuming the same transform location as before:
        quaternion_difference = tf.transformations.quaternion_about_axis(-1.57079633, (0, 0, 1))
        new_quaternion = tf.transformations.quaternion_multiply([0.000, 0.000, -0.501, 0.865], quaternion_difference)
        # LOL that was my previous position, so it went -90 from THERE, which was 180 from where I am now. <face palm>
        # Robot is now smarter than me . . . 
        goal.target_pose.pose.position.x = 1.725
        goal.target_pose.pose.position.y = 1.027
        goal.target_pose.pose.position.z = 0.101
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        
        # Testing to "zero"
        '''
        This is what I get when I first turn it on:
        rosrun tf tf_echo /odom /base_link
        - Translation: [0.000, 0.000, 0.101]
        - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
        '''

        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 0.101
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        # This work, it goes back home!
        
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
        
        #current_odom = self.currentOdom
        #print("New odom:")
        #print(current_odom.pose)
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        print "New Position: " + str(position)
        print "New Orientation: " + str(quaternion)

        rospy.loginfo("Ok, now what?")

if __name__ == '__main__':
    node = ArlobotExplore()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
