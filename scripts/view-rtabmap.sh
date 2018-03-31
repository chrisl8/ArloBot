#!/bin/bash
if [ $(jq '.cloudServer.exists' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    node ${HOME}/catkin_ws/src/ArloBot/node/addRobotIpToEtcHosts.js > /dev/null
    if [ $? -gt 0 ]
        then
        echo "You will be asked for your password in order to have root access to edit /etc/hosts"
        echo "to add the host/ip for your robot."
        export NVM_DIR="$HOME/.nvm"
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
        [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
        sudo ${HOME}/.nvm/versions/node/$(nvm version)/bin/node ${HOME}/catkin_ws/src/ArloBot/node/addRobotIpToEtcHosts.js
    fi
fi
echo "If you do not have a map loaded,"
echo "set Global Options->Fixed Frame to"
echo "'odom' in order to make this work."
#roslaunch arlobot_rviz_launchers view_navigation.launch --screen
roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start"
