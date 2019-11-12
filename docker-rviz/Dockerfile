FROM ros:melodic

SHELL ["/bin/bash", "-c"]

# ros desktop full for GUIs
RUN apt-get update && apt-get install -y \
        ros-melodic-desktop-full \
        ros-melodic-tf2-tools \
        git

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin_make

# clone Arlobot source required for rviz files
WORKDIR /catkin_ws/src
# This defeats the cache
ADD https://api.github.com/repos/chrisl8/ArloBot/compare/new-serial-interface...HEAD /dev/null
RUN git clone -b new-serial-interface https://github.com/chrisl8/ArloBot.git

# xpra crashes if the path of called apps is too long,
# so we link the ones we call externally to root.
# You can always use xterm to run other things if you want to.
WORKDIR /
RUN ln -s /catkin_ws/src/ArloBot/scripts/view-navigation.sh
RUN ln -s /catkin_ws/src/ArloBot/scripts/view-robot.sh

# initialize ROS (master uri, environments, etc.)
COPY docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]

# default command
CMD ["bash"]
