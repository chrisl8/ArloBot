# This will launch RVIZ and the ROS Robot Model
# on your local system so that you can view the robot
# model in RVIZ for tweaking the look and part location
# without having to run the entire ROS robot stack.
# I use this to work on the model from my desktop,
# instead of having to do it from the laptop on the robot.
export ROS_MASTER_URI=http://localhost:11311
export ARLOBOT_MODEL=$(jq '.arlobotModel' ${HOME}/.arlobot/personalDataForBehavior.json | tr -d '"')
roslaunch arlobot_bringup model_robot.launch
