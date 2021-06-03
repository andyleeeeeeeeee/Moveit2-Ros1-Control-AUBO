FROM ros:kinetic-robot-xenial

WORKDIR /root

# ==================================================================
# Install dependencies
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --allow-unauthenticated \
    vim \
    ros-kinetic-moveit \
    ros-kinetic-industrial-core \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --allow-unauthenticated \
    ros-kinetic-moveit-visual-tools \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws

COPY src /root/catkin_ws/src

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/kinetic/setup.bash \
    && cd /root/catkin_ws && catkin_make

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# RUN rm -fr /root/catkin_ws/src /root/catkin_ws/devel /root/catkin_ws/build

#ENTRYPOINT ["/bin/bash", "-c", "source /root/catkin_ws/install/setup.bash && roslaunch libnachi libnachi_node.launch"]
