FROM osrf/ros:foxy-desktop

WORKDIR v-hsc-ros-bridge/

# install ros build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions libopencv-dev liblua5.2-dev protobuf-compiler libprotobuf-dev && \
    rm -rf /var/lib/apt/lists/*

# clone ros package repo
RUN git clone \
      -b main \
      https://github.com/robocup-hl-tc/v-hsc-ros-bridge.git

# install ros package dependencies
RUN apt-get update && \
    rosdep install -y \
      --from-paths \
        v-hsc-ros-bridge/src/ros_bridge && \
    rm -rf /var/lib/apt/lists/*

# build ros package source
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    cd v-hsc-ros-bridge/ && \
    colcon build

RUN rosdep fix-permissions

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

RUN ["chmod", "+x", "v-hsc-ros-bridge/install/setup.bash"]

RUN bash v-hsc-ros-bridge/install/setup.bash

ENTRYPOINT ["bash", "-c", "cd v-hsc-ros-bridge/ && . install/setup.bash && ros2 run ros_bridge webots_controller"]