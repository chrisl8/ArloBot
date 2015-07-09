#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool
import move_base_msgs.msg
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from hector_nav_msgs.srv import GetRobotTrajectory # It says 'msgs' but it is a srv!
import tf
import math
from arlobot_msgs.srv import pause_explorer

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
        # This is taken care of later instead on a loop that checks the status before continuing.

        # We will decrement this number if the plan is failing,
        # to see if we can make it work by attempting a "closer" point along
        # the path.
        self._poseDivider = 1

        self._active_controller = ""
        rospy.Subscriber("/cmd_vel_mux/active", String, self._set_active_controller)  # Is this line or the below bad redundancy?

        # Create a service that can be called to Pause the Explorer
        # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
        self._explorer_paused = False
        pauseExplorer = rospy.Service('~pause_explorer', pause_explorer, self._pause_explorer)
        # /arlobot_explore/pause
        rospy.set_param("~pause", self._explorer_paused);

    def _pause_explorer(self, pause_explorer):
        self._explorer_paused = pause_explorer.pause_explorer
        rospy.set_param("~pause", self._explorer_paused);
        return self._explorer_paused

    def _set_active_controller(self, status):
        """
        Shut down the motors if the SafeToOperate topic goes false.
        Set unPlugging variable to allow for safe unplug operation.
        """
        self._active_controller = status.data
        rospy.loginfo(self._active_controller)

    def Stop(self):
        rospy.loginfo("ArlobotExplore is shutting down.")
        self._MoveBaseClient.cancel_goals_at_and_before_time(rospy.Time.now()) # In case the poor thing is still stuck trying to go nowhere!
        # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
        # Send a series of BE STILL commands just in case to make sure robot is left stationary
        rospy.loginfo("ArlobotExplore is sending twist commands to halt robot in case it is moving.")
        pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # You cannot just "send" this, you publish it, and you have to publish it repeatedly, or the robot never starts or just stops
        pub.publish(twist)
        count = 0
        while count < 2: # I'm not sure how many it takes, but more than one is best else sometimes it is missed.
            rospy.loginfo("Twist halt command " + str(count))
            pub.publish(twist)
            rospy.sleep(1)
            count += 1
        rospy.loginfo("ArlobotExplore robot halt should be complete now. Closing down.")

    def Run(self):
        # Waits until the action server has started up and started
        # listening for goals.
        '''
        This will stall until the move_base comes up,
        in other words, if you don't run gmapping before this, this will just wait,
        and it won't go on until gmapping says "odom received!"
        '''
        rospy.loginfo("Waiting for move_base to come up . . . ")
        self._MoveBaseClient.wait_for_server()
        rospy.loginfo("move_base is UP!")

        # Wait for tf_listener to be ready.
        # If you call self.tf_listener too soon it has no data in the listener buffer!
        # http://answers.ros.org/question/164911/move_base-and-extrapolation-errors-into-the-future/
        # We could put a static dealy in here, but this is faster.
        rospy.loginfo("Waiting for tf_listener to be ready . . . ")
        rospy.sleep(.1) # Give it an initial rest just in case ;)
        tf_listener_ready = False
        while not tf_listener_ready:
            try:
                t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
                position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
                tf_listener_ready = True
            except tf.ExtrapolationException:
                rospy.loginfo("tf_listener not ready . . . ")
                rospy.sleep(.1)
        rospy.loginfo("tf_listener READY!")

        # Need to insert this here:
        while not rospy.is_shutdown():
            if not self._explorer_paused:
                if self._poseDivider == 1:
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
                    rospy.loginfo("Rotating ArloBot to scan surroundings.")
                    # You cannot just "send" a command, you have to broadcast it and keep it up or the robot will eventually stop.
                    while count < 8: # 8 seconds seems about right at angular_speed = 1 to do just over 1 rotation normally
                        if not rospy.is_shutdown():
                            rospy.loginfo("Twist command " + str(count))
                            pub.publish(twist)
                            rospy.sleep(1) # 1 second intervals seems to work fine
                            # Too far apart and the robot will stop periodically,
                            # Too close and they just pass by before the robot finishes what it is doing.
                        count += 1
                    #rospy.sleep(1) # Let it spin for a while
                    # Stop
                    rospy.loginfo("Stopping survey rotation . . . ")
                    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                    # You cannot just "send" this, you publish it, and you have to publish it repeatedly, or the robot never starts or just stops
                    pub.publish(twist)
                    count = 0
                    while count < 2: # I'm not sure how many it takes, but more than one is best else sometimes it is missed.
                        rospy.loginfo("Twist halt command " + str(count))
                        pub.publish(twist)
                        rospy.sleep(1)
                        count += 1
                    rospy.loginfo("Survey complete.")

                # Get an explore path and use it.
                # Use to the hector_exploration_node service and get goals!
                # You don't have to initialize services!
                # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
                rospy.loginfo("Getting exploration path.")
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
                    rospy.loginfo("Pose Divider = " + str(self._poseDivider))
                    #print len(response.trajectory.poses)
                    #print (len(response.trajectory.poses) - 1) / self._poseDivider
                    i = response.trajectory.poses[(len(response.trajectory.poses) - 1) / self._poseDivider]
                    #print i
                    explorePosition = [0,0,0]
                    explorePosition[0] = i.pose.position.x
                    explorePosition[1] = i.pose.position.y
                    explorePosition[2] = i.pose.position.z
                    exploreQuaternion = [0,0,0,0]
                    exploreQuaternion[0] = i.pose.orientation.x
                    exploreQuaternion[1] = i.pose.orientation.y
                    exploreQuaternion[2] = i.pose.orientation.z
                    exploreQuaternion[3] = i.pose.orientation.w
                    rospy.loginfo("Sending exploration goal to move_base.")
                    result, resultText = self._movetoPositiononMap(explorePosition, exploreQuaternion, 20)
                    '''
                    Timeout?
                    If I make it too long the robot can sit blocked for a long time,
                    but if I make it too short than long routes get preempted.

                    20 seconds seems to be about right, not cutting off any but the
                    longest paths, and they usually repeat anyway.
                    '''
                    rospy.loginfo("Final result: " + str(result) + " " + resultText)
                    # Each time we fail to reach the goal given by the hector explore planner,
                    # cut the subsequent goals in half,
                    # and keep cutting them in half again,
                    # until we come up with a goal that is close enough to reach.
                    # Then start over!
                    if result == GoalStatus.SUCCEEDED:
                        self._poseDivider = 1
                    else:
                        self._poseDivider += 1
                except:
                    rospy.loginfo("Hector Explore failed.")
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
            else:
                rospy.loginfo('Explorer Paused')
                rospy.sleep(1)
        #t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        #position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        #print "Final Position: " + str(position)
        #print "Final Orientation: " + str(quaternion)

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
                    finished = True
                    rospy.loginfo("Time-out reached while attempting to reach goal, canceling!")
                if count > 5 and self._active_controller == "idle":
                    finished = True
                    rospy.loginfo("Navigation is idle, canceling!")
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
                if self._explorer_paused:
                    finished = True
                    resultText = "PAUSED"
                rospy.loginfo("Pending result:" + str(result) + " " + resultText + " Time-out in :" + str(timeoutSeconds - count))
                # If it was determined that we are "finished" then cancel
                # any pending goals right now, because the loop will not
                # repeat.
                if finished:
                    # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
                    self._MoveBaseClient.cancel_goal()
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
