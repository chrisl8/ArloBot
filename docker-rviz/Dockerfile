FROM ros:noetic

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
# ros desktop full for GUIs
RUN apt-get update && apt-get install -y \
        ros-noetic-desktop-full \
        ros-noetic-tf2-tools \
        ros-noetic-slam-toolbox \
        git \
        xterm
ENV DEBIAN_FRONTEND=newt

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make

# clone Arlobot source required for rviz files
WORKDIR /catkin_ws/src
# This defeats the cache
ADD https://api.github.com/repos/chrisl8/ArloBot/compare/noetic...HEAD /dev/null
RUN git clone -b noetic https://github.com/chrisl8/ArloBot.git

# xpra crashes if the path of called apps is too long,
# so we link the ones we call externally to root.
# You can always use xterm to run other things if you want to.
WORKDIR /
RUN ln -s /catkin_ws/src/ArloBot/scripts/view-navigation.sh
RUN ln -s /catkin_ws/src/ArloBot/scripts/view-robot.sh
RUN ln -s /catkin_ws/src/ArloBot/scripts/runReconfigure.sh
RUN ln -s /catkin_ws/src/ArloBot/scripts/view-all-sources.sh

# initialize ROS (master uri, environments, etc.)
COPY docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]

# default command
CMD ["bash"]
