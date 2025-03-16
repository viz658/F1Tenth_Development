#!/bin/bash

NODE1=$1
NODE2=$2

# cd /sim_ws 
# colcon build 
# source /opt/ros/foxy/setup.bash                           
# source install/local_setup.bash 

if [ "$#" -eq 1 ]; then
    echo "Running node: $NODE1"
    tmux send-keys -t MySession:0.2 "./auto_check_nodes.sh $NODE1" Enter 
    ros2 run $NODE1 $NODE1
elif [ "$#" -eq 2 ]; then
    echo "Running node: $NODE1, $NODE2"
    tmux send-keys -t MySession:0.2 "./auto_check_nodes.sh $NODE1 $NODE2" Enter 
    ros2 run $NODE1 $NODE2
else
    echo "Invalid Num of Inputs"
fi 