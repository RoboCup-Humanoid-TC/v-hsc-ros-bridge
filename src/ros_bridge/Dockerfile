FROM osrf/ros:foxy-desktop

WORKDIR /opt/v-hsc-ros-bridge/

# install ros build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions libopencv-dev liblua5.2-dev protobuf-compiler libprotobuf-dev && \
    rm -rf /var/lib/apt/lists/*

# clone ros package repo
RUN git -C /opt clone \
      -b main \
      https://github.com/robocup-hl-tc/v-hsc-ros-bridge.git

# install ros package dependencies
RUN apt-get update && \
    rosdep install -y \
      --from-paths \
        /opt/v-hsc-ros-bridge/src/ros_bridge && \
    rm -rf /var/lib/apt/lists/*

# build ros package source
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        ros_bridge

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"'

CMD ["ros2", "run", "ros_bridge", "webots_controller"]
