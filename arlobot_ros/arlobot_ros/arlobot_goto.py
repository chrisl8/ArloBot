#!/usr/bin/env python
import rospy
import move_base_msgs.msg
from nav_msgs.msg import Odometry
import tf2_ros
from arlobot_ros.srv import GoToGoal

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

"""
This script should start a service,
which allows you to send arbitrary map based
goals to the robot.
It is left to another program to collect and save those
coordinates.

Call this like so:
rosservice call /arlobot_goto/GoToGoal 'pose: { position: { x: -0.136, y: 0.076, z: 0.101 }, orientation: { x: 0.000, y: 0.000, z: 0.017, w: 1.000} }'
"""


class ArlobotGoTo(object):
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node("arlobot_goto")
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber

        # Creates the SimpleActionClient, passing the type of the action
        self._MoveBaseClient = actionlib.SimpleActionClient(
            "move_base", move_base_msgs.msg.MoveBaseAction
        )

        # Listen to the transforms http://wiki.ros.org/tf/TfUsingPython
        self.tf_Buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_Buffer)

        # Global variable to hold current pose
        self.currentOdom = Odometry()

        # Subscribe to the current pose via odometry and populate our own variable with the data
        node.create_subscription(Odometry, "odom", self._SetCurrentOdom)
        # Turns out this works great if you have no map and just want to make movements based on odometry,
        # but if you are using a map, you need the /map to /base_link transform!

        # Create a service that can be called to send robot to a map based goal
        # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
        node.create_service(GoToGoal, "~GoToGoal", self._go_to_goal)

        rospy.spin()
        # Now we just wait for someone to call us!

    def _SetCurrentOdom(self, currentOdom):
        self.currentOdom = currentOdom

    def _go_to_goal(self, new_goal):

        # Waits until the action server has started up and started
        # listening for goals.
        """
        This will stall until the move_base comes up,
        in other words, if you don't run move_base before this, this will just wait,
        and it won't go on until move_base says "odom received!"
        """
        node.get_logger().info("Waiting for move_base to come up . . . ")
        self._MoveBaseClient.wait_for_server()
        node.get_logger().info("move_base is UP!")

        # Wait for tf_listener to be ready.
        # NOTE: I'm not sure this is required anymore or not
        # If you call self.tf_listener too soon it has no data in the listener buffer!
        # http://answers.ros.org/question/164911/move_base-and-extrapolation-errors-into-the-future/
        # We could put a static delay in here, but this is faster.
        node.get_logger().info("Waiting for tf_listener to be ready . . . ")
        tf_listener_ready = False
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        while not tf_listener_ready:
            try:
                self.tf_Buffer.lookup_transform("map", "base_link", rospy.Time())
                tf_listener_ready = True
            except tf2_ros.ExtrapolationException:
                node.get_logger().info("tf_listener not ready . . . ")
                rospy.sleep(0.1)
        node.get_logger().info("tf_listener READY!")

        # Create a variable to hold our goal
        goal = move_base_msgs.msg.MoveBaseGoal()
        # Note that move_base will not go to an all zero target.

        # we'll create a goal to send to move_base
        # If you are just sending commands to the robot with no map use base_link
        # goal.target_pose.header.frame_id = "base_link"
        # But if you have SLAM or Localization active and are using a map, you need to use the map!
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose = new_goal.pose

        node.get_logger().info("Populated goal.")

        #######################################
        # This is the part that sends the goal!#
        #######################################

        node.get_logger().info("Sending goal")
        # Sends the goal to the action server.
        result = -1
        timeoutSeconds = 60  # TODO: Should this be sent as part of the call?
        if rclpy.ok():
            self._MoveBaseClient.cancel_goals_at_and_before_time(rospy.Time.now())
            # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
        goal.target_pose.header.stamp = rospy.Time.now()
        if rclpy.ok():
            self._MoveBaseClient.send_goal(goal)
            count = 0
            finished = False
            # Wait for action to finish or timeout to run out
            # Use double timeout, but cancel if timeout is met
            while (
                count < (timeoutSeconds * 2)
                and not finished
                and rclpy.ok()
            ):
                if count > timeoutSeconds:
                    finished = True
                    node.get_logger().info(
                        "Time-out reached while attempting to reach goal, canceling!"
                    )
                # NOTE: If the robot tends to get stuck without moving at all,
                #       1. Subscribe to cmd_vel
                #       2. Increment a timer.
                #       3. Zero it out whenever cmd_vel is updated.
                #       4. Cancel this if the timer gets too high.
                count += 1
                rospy.sleep(
                    1
                )  # Set this delay as you see fit. If the robot is extremely fast this could be slowing you down!
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
                node.get_logger().info(
                    "Pending result:"
                    + str(result)
                    + " "
                    + resultText
                    + " Time-out in :"
                    + str(timeoutSeconds - count)
                )
                # If it was determined that we are "finished" then cancel
                # any pending goals right now, because the loop will not
                # repeat.
                if finished:
                    # NOTE: Do not use cancel_all_goals here as it can cancel future goals sometimes!
                    self._MoveBaseClient.cancel_goal()

        trans = self.tf_Buffer.lookup_transform("map", "base_link", rospy.Time())

        node.get_logger().info("New Position: ")
        node.get_logger().info(str(trans.transform.translation))
        node.get_logger().info(" New Orientation: ")
        node.get_logger().info(str(trans.transform.rotation))

        if result == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def Stop(self):
        node.get_logger().info("Arlobot Goto is shutting down.")
        self._MoveBaseClient.cancel_goals_at_and_before_time(rospy.Time.now())

    def Run(self):
        print("This is a ROS node.")


if __name__ == "__main__":
    node = ArlobotGoTo()
    rospy.on_shutdown(node.Stop)
    try:
        node.Run()
    except rospy.ROSInterruptException:
        node.Stop()
