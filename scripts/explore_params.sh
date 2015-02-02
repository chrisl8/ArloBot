rosparam set /hector_exploration_node/hector_exploration_planner/dist_for_goal_reached 0.50
#rosparam set /hector_exploration_node/hector_exploration_planner/goal_angle_penalty
#rosparam set /hector_exploration_node/hector_exploration_planner/min_frontier_size
rosparam set /hector_exploration_node/hector_exploration_planner/min_obstacle_dist 20
rosparam set /hector_exploration_node/hector_exploration_planner/obstacle_cutoff_distance 2.0
#rosparam set /hector_exploration_node/hector_exploration_planner/plan_in_unknown
#rosparam set /hector_exploration_node/hector_exploration_planner/same_frontier_distance
#rosparam set /hector_exploration_node/hector_exploration_planner/security_constant
#rosparam set /hector_exploration_node/hector_exploration_planner/use_inflated_obstacles
echo -n "dist for goal reached: "
rosparam get /hector_exploration_node/hector_exploration_planner/dist_for_goal_reached
echo -n "goal_angle_penalty: "
rosparam get /hector_exploration_node/hector_exploration_planner/goal_angle_penalty
echo -n "min_frontier_size: "
rosparam get /hector_exploration_node/hector_exploration_planner/min_frontier_size
echo -n "min_obstacle_dist: "
rosparam get /hector_exploration_node/hector_exploration_planner/min_obstacle_dist
echo -n "obstacle_cutoff_distance: "
rosparam get /hector_exploration_node/hector_exploration_planner/obstacle_cutoff_distance
echo -n "plan_in_unknown: "
rosparam get /hector_exploration_node/hector_exploration_planner/plan_in_unknown
echo -n "same_frontier_distance: "
rosparam get /hector_exploration_node/hector_exploration_planner/same_frontier_distance
echo -n "security_constant: "
rosparam get /hector_exploration_node/hector_exploration_planner/security_constant
echo -n "use_inflated_obstacles: "
rosparam get /hector_exploration_node/hector_exploration_planner/use_inflated_obstacles

