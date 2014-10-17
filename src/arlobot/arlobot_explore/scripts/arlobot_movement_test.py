#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool
import move_base_msgs.msg
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from hector_nav_msgs.srv import GetRobotTrajectory # It says 'msgs' but it is a srv!
import tf
import math

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus

'''
An attempt at "autonomous" navigation.
See:
http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
'''

'''
Some info on transforms and quaternions:
http://answers.ros.org/question/159171/how-to-get-robot-position-xy-in-a-map/
http://answers.ros.org/question/67206/getting-accurate-real-time-xy-coordinates-with-gmapping/
http://wiki.ros.org/geometry/RotationMethods
http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/program/index.htm
http://demonstrations.wolfram.com/FromQuaternionTo3DRotation/
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
        #rospy.sleep(2) # If you call self.tf_listener too soon it has no data in the listener buffer!
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
        self._MoveBaseClient.cancel_goals_at_and_before_time(rospy.Time.now()) # In case the poor thing is still stuck trying to go nowhere!
        # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
        # Send a series of BE STILL commands just in case to make sure robot is left stationary
        print "Stopping . . . "
        pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        print "Sending twist command:"
        print twist
        # You cannot just "send" this, you publish it, and you have to publish it repeatedly, or the robot never starts or just stops
        pub.publish(twist)
        count = 0
        while count < 2: # I'm not sure how many it takes, but more than one is best else sometimes it is missed.
            print count
            print twist
            pub.publish(twist)
            rospy.sleep(1)
            count += 1
        print "twist based rotation done."
        
    def Run(self):
        # Waits until the action server has started up and started
        # listening for goals.
        '''
        This will stall until the move_base comes up,
        in other words, if you don't run gmapping before this, this will just wait,
        and it won't go on until gmapping says "odom received!"
        '''
        print "Waiting for move_base to come up . . . "
        self._MoveBaseClient.wait_for_server()
        rospy.loginfo("move_base is UP!")
        # Wait for tf_listener to be ready.
        # If you call self.tf_listener too soon it has no data in the listener buffer!
        # http://answers.ros.org/question/164911/move_base-and-extrapolation-errors-into-the-future/
        # We could put a static dealy in here, but this is faster.
        print "Waiting for tf_listener to be ready . . . "
        rospy.sleep(.1) # Give it an initial rest just in case ;)
        tf_listener_ready = False
        while not tf_listener_ready:
            try:
                t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
                position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
                tf_listener_ready = True
            except tf.ExtrapolationException:
                print "not ready . . . "
                rospy.sleep(.1)
        print "tf_listener READY!"
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
        
        # NOW LET'S GET THE CURRENT POSITION PROGRAMATICALLY!
        # http://wiki.ros.org/tf/TfUsingPython

        #(trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time())
        #if self.tf_listener.frameExists("/base_link") and self.tf_listener.frameExists("/map"):
        #print "Transforms:"
        #position, quaternion = self.tf_listener.lookupTransform("/base_link", "/map", t)
        #self.tf_listener.waitForTransform("/base_link","/map",rospy.Time.now(),rospy.Duration(5.0))
        #print "Position: " + str(position)
        #print "Orientation: " + str(quaternion)
        
        # Testing stay still
        '''
        If I get the CURRENT position and send it, it should stay still, right?
        rosrun tf tf_echo /map /base_link
        This works IF you set the goal.target_pose.header.frame_id = "map"
        '''
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)

        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        
        # It works!
        
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
        # Get current position:
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        #print(new_quaternion)
        #[0.000, 0.000, -0.422, 0.907]
        # Current location:
        #- Translation: [1.725, 1.027, 0.101]
        #- Rotation: in Quaternion [0.000, 0.000, -0.422, 0.907]

        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]

        # It works, we rotated left 90 degrees!

        # Rotate by -90 degrees assuming the same transform location as before:
        quaternion_difference = tf.transformations.quaternion_about_axis(-1.57079633, (0, 0, 1))
        # Get current position:
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]

        # Complete a circle by quarters:
        '''
        rotation_angle = -90 * math.pi / 180; # -90 degree
        quaternion_difference = tf.transformations.quaternion_about_axis(rotation_angle, (0, 0, 1))
        print "quaternion_difference: " + str(quaternion_difference)
        # 1
        # Get current position:
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        original_position = position
        original_quaternion = quaternion
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        rospy.sleep(1)
        # 2
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        rospy.sleep(1)
        # 3
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        rospy.sleep(1)
        # 4
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        
        # Each rotation is shy of 90
        # Possibly due to getting ahead of the transform data?

        # Return to original position:
        position = original_position
        quaternion = original_quaternion
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        '''
        
        # What if we calculate all 4 steps from the original data instead of the current location?
        '''
        This works a lot better :)
        '''
        '''
        rotation_angle = -90 * math.pi / 180; # -90 degree
        quaternion_difference = tf.transformations.quaternion_about_axis(rotation_angle, (0, 0, 1))
        print "quaternion_difference: " + str(quaternion_difference)
        # 1
        # Get current position:
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        original_position = position
        original_quaternion = quaternion
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        rospy.sleep(1)
        # 2
        #t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        #position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        quaternion = new_quaternion
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        rospy.sleep(1)
        # 3
        #t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        #position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        quaternion = new_quaternion
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        rospy.sleep(1)
        # 4
        #t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        #position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        quaternion = new_quaternion
        new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = new_quaternion[0]
        goal.target_pose.pose.orientation.y = new_quaternion[1]
        goal.target_pose.pose.orientation.z = new_quaternion[2]
        goal.target_pose.pose.orientation.w = new_quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        
        # Return to original position:
        position = original_position
        quaternion = original_quaternion
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        print quaternion
        print new_quaternion
        self._MoveBaseClient.send_goal(goal)
        rospy.loginfo("Waiting for response . . .");
        self._MoveBaseClient.wait_for_result()
        '''

        # Version of rotate full from arlobot_explore
        # Rotate in a circle, by 90 degree increments, using current location as the basis
        # Well tested, so please use this bit of code if you want to rotate in inecrements by map.
        # I may be using the free rotate below instead though.
        '''
        rospy.loginfo("Explorer rotating to scan area.")
        rotation_angle = -90 * math.pi / 180; # -90 degree
        quaternion_difference = tf.transformations.quaternion_about_axis(rotation_angle, (0, 0, 1))
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        original_position = position
        original_quaternion = quaternion
        count = 1
        while count < 5:
            print count
            new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
            rospy.loginfo("Sending first quarter rotation to move_base.")
            result, resultText = self._movetoPositiononMap(position, new_quaternion, 10)
            print "Final result: " + str(result) + " " + resultText
            quaternion = new_quaternion
            count += 1
        print "Return to original position"
        result, resultText = self._movetoPositiononMap(original_position, original_quaternion, 10)
        print "Final result: " + str(result) + " " + resultText
        '''
        
        # How about an uncontrolled rotation sent to cmd_vel instead?
        # If you publish to ~cmd_vel like the arlobot_teleop script does, then you have to remap that in your launch file like:
        # <remap from="arlobot_teleop_keyboard/cmd_vel" to="cmd_vel_mux/input/teleop"/>
        # or for this one:
        # <remap from="arlobot_explore/cmd_vel" to="cmd_vel_mux/input/teleop"/>
        # or just publish to cmd_vel_mux/input/teleop
        #pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
        twist = Twist()
        linear_speed = 0 # Rotate in place, no liner movement
        angular_speed = 1 # This can be positive or negative
        # Positive angular_speed will be left or counter-clockwise
        # Negative angular_speed will be right or clockwise
        # 1 is a nice calm speed that seems to allow the map to build
        # 2 is pretty fast, maybe a little scary and it sometimes gets off track!
        # I will test higher speeds in a more open area later. :)
        twist.linear.x = linear_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_speed
        count = 0
        print "Sending twist commands:"
        # You cannot just "send" a command, you have to broadcast it and keep it up or the robot will eventually stop.
        while count < 8: # 8 seconds seems about right at angular_speed = 1 to do just over 1 rotation normally
            print count
            print twist
            pub.publish(twist)
            rospy.sleep(1) # 1 second intervals seems to work fine
            # Too far apart and the robot will stop periodically,
            # Too close and they just pass by before the robot finishes what it is doing.
            count += 1
        #rospy.sleep(1) # Let it spin for a while
        # Stop
        print "Stopping . . . "
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        print "Sending twist command:"
        print twist
        # You cannot just "send" this, you publish it, and you have to publish it repeatedly, or the robot never starts or just stops
        pub.publish(twist)
        count = 0
        while count < 2: # I'm not sure how many it takes, but more than one is best else sometimes it is missed.
            print count
            print twist
            pub.publish(twist)
            rospy.sleep(1)
            count += 1
        print "twist based rotation done."

        # Get an explore path and use it.
        # Use to the hector_exploration_node service and get goals!
        # You don't have to initialize services!
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
        print "Getting exploration path."
        rospy.wait_for_service('get_exploration_path')
        try:
            get_exploration_path = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
            response = get_exploration_path()
            #rospy.loginfo("get_exploratin_path:")
            #rospy.loginfo(response)

            # The contents of this GetRobotTrajectory service is a nav_msgs/Path message
            # http://docs.ros.org/api/nav_msgs/html/msg/Path.html

            
            # Lets just use the LAST pose and let gmapping deal with the path
            #print(response.trajectory.poses[-1])
            i = response.trajectory.poses[-1]
            explorePosition = [0,0,0]
            explorePosition[0] = i.pose.position.x
            explorePosition[1] = i.pose.position.y
            explorePosition[2] = i.pose.position.z
            exploreQuaternion = [0,0,0,0]
            exploreQuaternion[0] = i.pose.orientation.x
            exploreQuaternion[1] = i.pose.orientation.y
            exploreQuaternion[2] = i.pose.orientation.z
            exploreQuaternion[3] = i.pose.orientation.w
            result, resultText = self._movetoPositiononMap(explorePosition, exploreQuaternion, 20)
            '''
            Timeout?
            If I make it too long the robot can sit blocked for a long time,
            but if I make it too short than long routes get preempted.
            
            20 seconds seems to be about right, not cutting off any but the
            longest paths, and they usually repeat anyway.
            '''
            print "Final result: " + str(result) + " " + resultText
        except:
            print "Hector Explore failed."
            
        '''
        IDEAS:
        It may work better to do an odom based rotation, although I'm not sure how to know when it is complete?
        I should adjust some of the planner and nav stack parameters to operate closer to my desires,
        exploring has a different set of needs for navigating a known map:
        ideas:
        /move_base/DWAPlannerROS/max_rot_vel # I have set this to 1.0, which makes it behave much more calmly!
        /move_base/clearing_rotation_allowed # I have disabled this, this makes a HUGE difference.
        /move_base/controller_frequency
        /move_base/controller_patience
        /move_base/recovery_behavior_enabled
        
        '''
        #print("Populated goal:")
        #print(goal.target_pose.pose)

        #rospy.loginfo("Sending goal");
        # Sends the goal to the action server.
        #self._MoveBaseClient.send_goal(goal)

        #rospy.loginfo("Waiting for response . . .");
        # Waits for the server to finish performing the action.
        #self._MoveBaseClient.wait_for_result()
        
        #This could wait a VERY long time,
        #if the move_base doesn't have a timeout it will never come back,
        #in most cases it does, but it seems in some cases it will retry forever.
        # http://docs.ros.org/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a460c9f52fd650f918cb287765f169445
        
        #result = self._MoveBaseClient.get_result()
        #rospy.loginfo(result)
        #result = self._MoveBaseClient.get_state()
        #rospy.loginfo(result)
        
        #current_odom = self.currentOdom
        #print("New odom:")
        #print(current_odom.pose)
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        print "Final Position: " + str(position)
        print "Final Orientation: " + str(quaternion)

        rospy.loginfo("Clean Finish")
        
    def _movetoPositiononMap(self, position, quaternion, timeoutSeconds):
        result = -1
        resultText = ""
        if not rospy.is_shutdown():
            self._MoveBaseClient.cancel_goals_at_and_before_time(rospy.Time.now())
            # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        goal.target_pose.header.stamp = rospy.Time.now()
        if not rospy.is_shutdown():
            self._MoveBaseClient.send_goal(goal)
            count = 0
            finished = False
            # Wait for action to finish or timeout to run out
            # Use double timeout, but cancel if timeout is met
            while count < (timeoutSeconds * 2) and not finished and not rospy.is_shutdown():
                if count > timeoutSeconds:
                    self._MoveBaseClient.cancel_goal()
                    # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
                    print "Timeout reached, canceling!"
                count += 1
                rospy.sleep(1) # Set this delay as you see fit. If the robot is extremely fast this could be slowing you down!
                result = self._MoveBaseClient.get_state()
                resultText = ""
                # http://docs.ros.org/indigo/api/actionlib_msgs/html/msg/GoalStatus.html
                if result == GoalStatus.PENDING:
                    resultText = "PENDING"
                if result == GoalStatus.ACTIVE:
                    resultText = "ACTIVE"
                if result == GoalStatus.PREEMPTED:
                    finished = True
                    resultText = "PREEMPTED"
                if result == GoalStatus.SUCCEEDED:
                    finished = True
                    resultText = "SUCCEEDED"
                if result == GoalStatus.ABORTED:
                    finished = True
                    resultText = "ABORTED"
                if result == GoalStatus.REJECTED:
                    finished = True
                    resultText = "REJECTED"
                if result == GoalStatus.PREEMPTING:
                    resultText = "PREEMPTING"
                if result == GoalStatus.RECALLING:
                    resultText = "RECALLING"
                if result == GoalStatus.RECALLED:
                    finished = True
                    resultText = "RECALLED"
                if result == GoalStatus.LOST:
                    finished = True
                    resultText = "LOST"
                print str(result) + " " + resultText
        #self._MoveBaseClient.send_goal(goal)
        #rospy.loginfo("Waiting for response . . .");
        #self._MoveBaseClient.wait_for_result()
        return result, resultText
        

if __name__ == '__main__':
    node = ArlobotExplore()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
