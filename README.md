# v-hsc-ros-bridge
This repository provides a ROS package named `ros_bridge` that can be used to communicate with the Webots server while using the Player/Client API.

It requests and receives sensor data (specifed int the `src/ros_bridge/resources/devices.json`) over specific topics and subscribes to actuator command messages. Node `webots_controller` is responsible for these operations.