# ROS Tuning Summary
This document gives an excellent explanation of tuning ROS parameters:
http://kaiyuzheng.me/documents/navguide.pdf

This summary assumes you’ve read that. I’m just consolidating some information from that document.

NOTE: The only way to find all of the parameters is to use rqt_reconfigure, because many parameters are not in the files, but just set to defaults.  
`rosrun rqt_reconfigure rqt_reconfigure`  
or  
`~/catkin_ws/src/ArloBot/scripts/runReconfigure.sh`  

## Key
*filename*  
`settings inside file`

## Robot Dimensions
*costmap_common_params.yaml*  
`robot_radius: 0.22545`

## Velocity
NOTE: If your robot is always overshooting goals, especially rotationally, your acceleration parameters are probably wrong. Read the above doc for some insight on how to test and tune those.

*dwa_local_planner_params.yaml*  
The top of the file has all of the max/min speed and acceleration settings.

## Path Planning
### Set the planners to use
*move_base_params.yaml*  
`base_local_planner: "dwa_local_planner/DWAPlannerROS"`
`base_global_planner: "global_planner/GlobalPlanner"`

### Global defaults to keep
*global_planner_params.yaml*  
`old_navfn_behavior: false`  
`use_quadratic: true`  
`use_dijkstra: true`  
`use_grid_path: false`

### Global path tuning
*global_planner_params.yaml*  
See page 5 for good pictures and 6 for warnings.  
`lethal_cost: 253`  
`neutral_cost: 66` #NOTE: I wonder if setting this to 1 would make it take wider corners?  
`cost_factor: 0.55`

### Local planning (DWA)
*dwa_local_planner_params.yaml*

`sim_time: 4.0`  
Sim Time is how far into the future DWA simulates each possible path. High values are CPU intensive.   
Too low and the paths created are not of good quality.  
Too high and it creates long inflexible paths, although it keeps updating them so it should work.  
Turtlebot uses like 1.7. Page 8 of the referenced PDF suggests 4.  
**On Arlobot, a high sim_time seems to work if the robot is headed in the right direction.
However, a good way to show the problems is start the robot facing in the opposite direction:
It starts to rotate toward the global path, and then at some point, it plans a wide arc to “drive to” the global path, instead of just rotating to it. This usually throws the robot out into the room where it is colliding with everything.
So a very short sim_time, like 1.0, seems to keep it just sticking to the global path.**

### Velocity Samples (Page 9)
*dwa_local_planner_params.yaml*  
`vx_samples: 20`  
`vy_samples: 1 # This don’t apply if your robot cannot move sideways.`  
`vtheta_samples: 40`    
These can increase CPU load, but can improve quality. See Turtlebot’s parameters for a minimum.  
`sim_granularity: 0.025` # Not in the file, but probably in rqtreconfigure. Default is fine. See Page 9 for details.

### Local Planner Trajectory Scoring (Page 9)
*dwa_local_planner_params.yaml*  

As of Melodic the defaults for path_distance_bias, goal_distance_bias, and occdist_scale have changed due to bugs in how the previous versions calculated
https://github.com/ros-planning/navigation/pull/763/files

`#path_distance_bias: 64.0 # How closely to follow the global path.`  
`#goal_distance_bias: 20.0 # Weight given to goal regardless of path`  
`#occdist_scale: 0.02 # Weight given to avoiding obstacles`  
Too high of occdist_scale will cause the robot to cycle indecisively.  
With a low sim time, a path_distance_bias of 64.0 seems to work.  
Higher helps in some situations, but in others it will spend more time getting “back to the path” than moving along it.

`oscillation_reset_dist: 0.05  # 0.05   - how far to travel before resetting oscillation flags`  
If the robot is not getting anywhere, it will initiate recovery behaviors.  
This is the distance it must progress to be considered “not oscillating” anymore.

### Final goal tolerances
*dwa_local_planner_params.yaml*  
`yaw_goal_tolerance: 0.3  # 0.05`  
`xy_goal_tolerance: 0.15  # 0.10`  
`# latch_xy_goal_tolerance: false`  

### Inflation
This is how we get that blue coverage of the local planner area to push the robot into the middle of the room. See page 12 for an excellent picture.
*costmap_common_params.yaml*  
```
inflation_layer:
  enabled:              true
  cost_scaling_factor:  2.58
  inflation_radius:     1.75
```

### Resolution (Page 12)
*global_costmap_params.yaml*  
*local_costmap_params.yaml*  
`resolution: 0.01`  
In theory setting it too low can cause gaps of “unknown” space in the maps, because the laser cannot fill them in,  
but when I set it to `0.05` I got gaps in the wall instead!

### Recovery behaviors
