#!/bin/bash

NODE1=$1
NODE2=$2
last_mod_time=0

if [ "$#" -eq 1 ]; then
    echo "Watching for changes in /sim_ws/src/nodes for node: $NODE1"
    current_mod_time=$(find /sim_ws/src/nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
    last_mod_time="$current_mod_time"
    
    while true; do
        current_mod_time=$(find /sim_ws/src/nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
        if (( $(echo "$current_mod_time > $last_mod_time" | bc -l) )); then
            echo "Change detected. Restarting session..."
            tmux kill-session
        fi
        sleep 1
    done
elif [ "$#" -eq 2 ]; then
    echo "Watching for changes in /sim_ws/src/nodes for node: $NODE1"
    current_mod_time=$(find /sim_ws/src/nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
    last_mod_time="$current_mod_time"

    while true; do
        current_mod_time=$(find /sim_ws/src/nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
        if (( $(echo "$current_mod_time > $last_mod_time" | bc -l) )); then
            echo "Change detected. Restarting session..."
            tmux kill-session
        fi
        sleep 1
    done
else
    echo "Invalid Num of Inputs"
    exit 1
fi
