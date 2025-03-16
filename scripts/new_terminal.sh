#!/bin/bash


NODE1=$1
NODE2=$2

#run docker exec -it f1tenth_gym_ros-sim-1 /bin/bash -c "./new_terminal.sh"
cd /sim_ws 
colcon build 
source /opt/ros/foxy/setup.bash                           
source install/local_setup.bash                          

tmux start-server
# create a session with four panes
tmux new-session -d -s MySession -n Shell1 "/bin/bash -i"
tmux split-window -t MySession:0 "/bin/bash -i"
tmux split-window -t MySession:0 "/bin/bash -i"
tmux split-window -t MySession:0 "/bin/bash -i"

tmux select-layout -t MySession:0 tiled
sleep 1
tmux send-keys -t MySession:0.0 "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" Enter  

if [ $# -eq 1 ]; then
    tmux send-keys -t MySession:0.1 "./run_node.sh $NODE1" Enter
elif [ $# -eq 2 ]; then
    tmux send-keys -t MySession:0.1 "./run_node.sh $NODE1 $NODE2" Enter
fi

tmux attach -t MySession
