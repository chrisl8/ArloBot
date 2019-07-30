#!/usr/bin/env bash

# This will set the pose on whatever map is loaded to the 0 position.
# This is already done by ROS when you load a map,
# But this can be useful for resetting for testing, or
# you can use this example to set other poses.
rostopic pub -1 initialpose geometry_msgs/PoseWithCovarianceStamped "{ header: { seq: 0, stamp: { secs: 0, nsecs: 0  }, frame_id: map  }, pose: { pose: { position: { x: 0.0, y: 0.0, z: 0.0  }, orientation: { x: 0.0, y: 0.0, z: -0.607, w: 0.0 } } , covariance: [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  ]  }  }"
# Change the parts in position: { x: 0, y: 0, z: 0.0  }, orientation: { x: 0.0, y: 0.0, z: -0.607, w: 0.0  }
# to affect the robot's postion on the map
# If you get errors, check the format. It is VERY picky about spaces and such!
# Also, don't try to set your z axis on the orientation to 0.0, it breaks everything. Not sure why.:q
