#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Bool
import move_base_msgs.msg
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from hector_nav_msgs.srv import GetRobotTrajectory # It says 'msgs' but it is a srv!
import tf
import math
from arlobot_msgs.srv import go_to_goal

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus

'''
This script should start a service,
which allows you to send arbitrary map based
goals to the robot.
It is left to another program to collect and save those
coordinates.

Call this like so:
rosservice call /arlobot_goto/go_to_goal 'pose: { position: { x: -0.136, y: 0.076, z: 0.101 }, orientation: { x: 0.000, y: 0.000, z: 0.017, w: 1.000} }'
'''

class ArlobotGoTo(object):

#TODO: Test for robot movement before sending it places! It could get really goofy to send to -90 from it's position ten seconds ago!

    def __init__(self):
        rospy.init_node('arlobot_goto')
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        #self.r = rospy.Rate(1) # 1hz refresh rate

        # Creates the SimpleActionClient, passing the type of the action
        self._MoveBaseClient = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

        # Listen to the transforms http://wiki.ros.org/tf/TfUsingPython
        self.tf_listener = tf.listener.TransformListener()
        #rospy.sleep(2) # If you call self.tf_listener too soon it has no data in the listener buffer!
        # http://answers.ros.org/question/164911/move_base-and-extrapolation-errors-into-the-future/
        # This is taken care of later instead on a loop that checks the status before continuing.

        # Global variable to hold current pose
        self.currentOdom = Odometry()

        # Subscribe to the current pose via odometry and populate our own variable with the data
        '''
        I'm not sure of any other way to do this. I'd like to just "grab" it at a point in time, but subscriptions don't work taht way.
        '''
        rospy.Subscriber("odom", Odometry, self._SetCurrentOdom)
        # Turns out this works great if you have no map and just want to make movements based on odometry,
        # but if you are using a map, you need the /map to /base_link transform!

        # Subscribe to the controller topic to check to see if the robot is
        # actually IDLE even though we THINK we are going somewhere!
        self._active_controller = ""
        rospy.Subscriber("/cmd_vel_mux/active", String, self._set_active_controller)  # Is this line or the below bad redundancy?

        # Create a service that can be called to send robot to a map based goal
        # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
        goToGoal = rospy.Service('~go_to_goal', go_to_goal, self._go_to_goal)

        rospy.spin(); # Now we just wait for someone to call us!

    def _SetCurrentOdom(self, currentOdom):
        self.currentOdom = currentOdom

    def _set_active_controller(self, status):
        """
        Shut down the motors if the SafeToOperate topic goes false.
        Set unPlugging variable to allow for safe unplug operation.
        """
        self._active_controller = status.data
        rospy.loginfo('Active Controller: ' + self._active_controller)

    def _go_to_goal(self, new_goal):

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

        # Create a variable to hold our goal
        goal = move_base_msgs.msg.MoveBaseGoal()
        # Note that move_base will not go to an all zero target.

        #we'll create a goal to send to move_base
        # If you are just sending commands to the robot with no map use base_link
        #goal.target_pose.header.frame_id = "base_link"
        # But if you have gmapping active and are using a map, you need to use the map!
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose = new_goal.pose;

        rospy.loginfo("Populated goal.")

        #######################################
        #This is the part that sends the goal!#
        #######################################

        rospy.loginfo("Sending goal");
        # Sends the goal to the action server.
        result = -1
        resultText = ""
        timeoutSeconds = 60 # TODO: Should this be sent as part of the call?
        if not rospy.is_shutdown():
            self._MoveBaseClient.cancel_goals_at_and_before_time(rospy.Time.now())
            # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
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
                rospy.loginfo("Pending result:" + str(result) + " " + resultText + " Time-out in :" + str(timeoutSeconds - count))
                # If it was determined that we are "finished" then cancel
                # any pending goals right now, because the loop will not
                # repeat.
                if finished:
                    # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
                    self._MoveBaseClient.cancel_goal()

        #current_odom = self.currentOdom
        t = self.tf_listener.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf_listener.lookupTransform("/map", "/base_link", t)
        rospy.loginfo("New Position: " + str(position) + " New Orientation: " + str(quaternion))

        if result == GoalStatus.SUCCEEDED:
            return(True);
        else:
            return(False);

    def Stop(self, exception):
        #print(exception);
        rospy.loginfo("Arlobot Goto is shutting down.")
        self._MoveBaseClient.cancel_goals_at_and_before_time(rospy.Time.now())
        # In case the poor thing is still stuck trying to go nowhere!
        # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
        # Send a series of BE STILL commands just in case to make sure robot is left stationary

    def Run(self):
        print('This is a ROS node.');

if __name__ == '__main__':
    node = ArlobotGoTo()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
