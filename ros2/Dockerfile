FROM osrf/ros:jazzy-desktop-full-noble

SHELL ["/bin/bash", "-c"]

WORKDIR /root/ros2_ws

RUN apt-get update && apt-get install -y \
    libreadline-dev \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-mavros && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
