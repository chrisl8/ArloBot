#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool
import move_base_msgs.msg
from nav_msgs.msg import Path
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
        
    def Stop(self):
        rospy.loginfo("ArlobotExplore is shutting down.")
        self._MoveBaseClient.cancel_all_goals()
        
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
        #rospy.loginfo("Empty goal:")
        #rospy.loginfo(goal)
        
        goal.target_pose.header.frame_id = "map"

        while not rospy.is_shutdown():
            # Rotate in a circle, by 90 degree increments, using current location as the basis
            rospy.loginfo("Explorer rotating to scan area.")
            '''
            This works a lot better :)
            '''
            rotation_angle = -90 * math.pi / 180; # -90 degree
            quaternion_difference = tf.transformations.quaternion_about_axis(rotation_angle, (0, 0, 1))
            #print "quaternion_difference: " + str(quaternion_difference)
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
            rospy.loginfo("Sending first quarter rotation to move_base.")
            self._MoveBaseClient.cancel_all_goals()
            goal.target_pose.header.stamp = rospy.Time.now()
            if not rospy.is_shutdown():
                self._MoveBaseClient.send_goal_and_wait(goal, rospy.Duration(20), rospy.Duration(60)) # Limit how long it can take
            #self._MoveBaseClient.send_goal(goal)
            #rospy.loginfo("Waiting for response . . .");
            #self._MoveBaseClient.wait_for_result()
            rospy.sleep(1)
            # 2
            quaternion = new_quaternion
            new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
            goal.target_pose.pose.position.x = position[0]
            goal.target_pose.pose.position.y = position[1]
            goal.target_pose.pose.position.z = position[2]
            goal.target_pose.pose.orientation.x = new_quaternion[0]
            goal.target_pose.pose.orientation.y = new_quaternion[1]
            goal.target_pose.pose.orientation.z = new_quaternion[2]
            goal.target_pose.pose.orientation.w = new_quaternion[3]
            rospy.loginfo("Sending second quarter rotation to move_base.")
            self._MoveBaseClient.cancel_all_goals()
            goal.target_pose.header.stamp = rospy.Time.now()
            if not rospy.is_shutdown():
                self._MoveBaseClient.send_goal_and_wait(goal, rospy.Duration(20), rospy.Duration(60)) # Limit how long it can take
            #self._MoveBaseClient.send_goal(goal)
            #rospy.loginfo("Waiting for response . . .");
            #self._MoveBaseClient.wait_for_result()
            rospy.sleep(1)
            # 3
            quaternion = new_quaternion
            new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
            goal.target_pose.pose.position.x = position[0]
            goal.target_pose.pose.position.y = position[1]
            goal.target_pose.pose.position.z = position[2]
            goal.target_pose.pose.orientation.x = new_quaternion[0]
            goal.target_pose.pose.orientation.y = new_quaternion[1]
            goal.target_pose.pose.orientation.z = new_quaternion[2]
            goal.target_pose.pose.orientation.w = new_quaternion[3]
            rospy.loginfo("Sending third quarter rotation to move_base.")
            self._MoveBaseClient.cancel_all_goals()
            goal.target_pose.header.stamp = rospy.Time.now()
            if not rospy.is_shutdown():
                self._MoveBaseClient.send_goal_and_wait(goal, rospy.Duration(20), rospy.Duration(60)) # Limit how long it can take
            #self._MoveBaseClient.send_goal(goal)
            #rospy.loginfo("Waiting for response . . .");
            #self._MoveBaseClient.wait_for_result()
            rospy.sleep(1)
            # 4
            quaternion = new_quaternion
            new_quaternion = tf.transformations.quaternion_multiply(quaternion, quaternion_difference)
            goal.target_pose.pose.position.x = position[0]
            goal.target_pose.pose.position.y = position[1]
            goal.target_pose.pose.position.z = position[2]
            goal.target_pose.pose.orientation.x = new_quaternion[0]
            goal.target_pose.pose.orientation.y = new_quaternion[1]
            goal.target_pose.pose.orientation.z = new_quaternion[2]
            goal.target_pose.pose.orientation.w = new_quaternion[3]
            rospy.loginfo("Sending fourth quarter rotation to move_base.")
            self._MoveBaseClient.cancel_all_goals()
            goal.target_pose.header.stamp = rospy.Time.now()
            if not rospy.is_shutdown():
                self._MoveBaseClient.send_goal_and_wait(goal, rospy.Duration(20), rospy.Duration(60)) # Limit how long it can take
            #self._MoveBaseClient.send_goal(goal)
            #rospy.loginfo("Waiting for response . . .");
            #self._MoveBaseClient.wait_for_result()
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
            rospy.loginfo("Returning to original position.")
            self._MoveBaseClient.cancel_all_goals()
            goal.target_pose.header.stamp = rospy.Time.now()
            if not rospy.is_shutdown():
                self._MoveBaseClient.send_goal_and_wait(goal, rospy.Duration(20), rospy.Duration(60)) # Limit how long it can take
            #self._MoveBaseClient.send_goal(goal)
            #rospy.loginfo("Waiting for response . . .");
            #self._MoveBaseClient.wait_for_result()

            # Use to the hector_exploration_node service and get goals!
            # You don't have to initialize services!
            # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
            rospy.wait_for_service('get_exploration_path')
            try:
                get_exploration_path = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
                response = get_exploration_path()
                #rospy.loginfo("get_exploratin_path:")
                #rospy.loginfo(response)
                '''
                The contents of this GetRobotTrajectory service is a nav_msgs/Path message
                http://docs.ros.org/api/nav_msgs/html/msg/Path.html
                
                '''
                '''
                rospy.loginfo("Parsing . . .")
                rospy.loginfo("Here are all of the poses:")
                count = 1
                for i in response.trajectory.poses:
                    if not rospy.is_shutdown():
                        rospy.loginfo("Pose " + str(count))
                        rospy.loginfo(i)
                        if count < 999: # Use this to just try some of the path instead of the entire list
                            rosNow = rospy.Time.now()
                            goal.target_pose.header.frame_id = "map"
                            goal.target_pose.header.stamp = rosNow
                            goal.target_pose.pose.position.x = i.pose.position.x
                            goal.target_pose.pose.position.y = i.pose.position.y
                            goal.target_pose.pose.position.z = i.pose.position.z
                            goal.target_pose.pose.orientation.x = i.pose.orientation.x
                            goal.target_pose.pose.orientation.y = i.pose.orientation.y
                            goal.target_pose.pose.orientation.z = i.pose.orientation.z
                            goal.target_pose.pose.orientation.w = i.pose.orientation.w
                            rospy.loginfo("Goal " + str(count))
                            rospy.loginfo(goal)
                            rospy.loginfo("Sending goal");
                            # Sends the goal to the action server.
                            #self._MoveBaseClient.cancel_all_goals()
                            #self._MoveBaseClient.send_goal_and_wait(goal, rospy.Duration(10), rospy.Duration(10))

                            #rospy.loginfo("Waiting for response . . .");
                            # Waits for the server to finish performing the action.
                            #self._MoveBaseClient.wait_for_result()
                        count += 1
                '''
                #rospy.loginfo(response.trajectory.poses)
                
                # Lets juse use the LAST pose and let gmapping deal with the path
                #print(response.trajectory.poses[-1])
                i = response.trajectory.poses[-1]
                rosNow = rospy.Time.now()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rosNow
                goal.target_pose.pose.position.x = i.pose.position.x
                goal.target_pose.pose.position.y = i.pose.position.y
                goal.target_pose.pose.position.z = i.pose.position.z
                goal.target_pose.pose.orientation.x = i.pose.orientation.x
                goal.target_pose.pose.orientation.y = i.pose.orientation.y
                goal.target_pose.pose.orientation.z = i.pose.orientation.z
                goal.target_pose.pose.orientation.w = i.pose.orientation.w
                rospy.loginfo("Sending explore goal");
                # Sends the goal to the action server.
                #self._MoveBaseClient.cancel_all_goals()
                self._MoveBaseClient.cancel_all_goals()
                goal.target_pose.header.stamp = rospy.Time.now()
                if not rospy.is_shutdown():
                    self._MoveBaseClient.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(10)) # Limit how long it can take
                # We will just get another goal if it doesnt' reach the first one.

                #rospy.loginfo("Waiting for response . . .");
                # Waits for the server to finish performing the action.
                #self._MoveBaseClient.wait_for_result()

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            '''
            rosNow = rospy.Time.now()
            #we'll send a goal to the robot to move 1 meter forward
            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.header.stamp = rosNow

            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.orientation.w = 1.0

            rospy.loginfo("Populated goal:")
            rospy.loginfo(goal)

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
            rospy.loginfo(result)
            result = self._MoveBaseClient.get_state()
            rospy.loginfo(result)
            result = self._MoveBaseClient.get_state()
            rospy.loginfo(result)
            '''

        rospy.loginfo("Exiting Explore!")

if __name__ == '__main__':
    node = ArlobotExplore()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
