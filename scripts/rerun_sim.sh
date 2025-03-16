#!/bin/bash
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash


# Check if the user provided the node name input
if [ "$#" -ne 1 ]; then
    echo "Enter a node"
    exit 1
fi

NODE=$1

# Run the ROS2 node
echo "Running node: $NODE"
ros2 run $NODE $NODE
