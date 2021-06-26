FROM osrf/ros:foxy-desktop

# install ros build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions libopencv-dev liblua5.2-dev protobuf-compiler libprotobuf-dev && \
    rm -rf /var/lib/apt/lists/*

# copy ros package contents
COPY src/ /v-hsc-ros-bridge/src/

# install ros package dependencies
RUN apt-get update && \
    rosdep install -y \
      --from-paths \
        /v-hsc-ros-bridge/src/ros_bridge && \
    rm -rf /var/lib/apt/lists/*

# build ros package
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    cd /v-hsc-ros-bridge/ && \
    colcon build

# resolve permission errors
RUN rosdep fix-permissions

# add ros sourcing to .bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# give executing permission to package setup.sh
RUN chmod +x /v-hsc-ros-bridge/install/setup.bash

# source the built package
RUN bash /v-hsc-ros-bridge/install/setup.bash

ENTRYPOINT ["bash", "-c", "cd v-hsc-ros-bridge/ && . install/setup.bash && ros2 run ros_bridge webots_controller \"$@\"", "--"]
