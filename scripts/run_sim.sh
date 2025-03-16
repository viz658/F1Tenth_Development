#!/bin/bash

NODE1=$1
NODE2=$2
DISPLAY_VAL=$3

if [ $# -eq 2 ]; then
    if [[ "$NODE2" == "novnc" || "$NODE2" == "local" ]]; then
        DISPLAY_VAL=$NODE2
        NODE2=""
    else
        echo "Enter where you want the simulation to be displayed: \"local\" or \"novnc\""
        echo "You can enter scripts/run_sim.sh <node> <display> or scripts/run_sim.sh <node1> <node2> <display>"
    fi
elif [ $# -eq 1 ]; then
    echo "Error: Not enough arguments"
    echo "Enter where you want the simulation to be displayed: \"local\" or \"novnc\""
    echo "You can enter scripts/run_sim.sh <node> <display> or scripts/run_sim.sh <node1> <node2> <display>"
fi



# if rocker_flag is empty
if [ "$DISPLAY_VAL" == "novnc" ]; then 
    echo "Simulation will be displayed in noVNC"
    echo "The sim_local & novnc container will be closed"
    docker compose stop sim_local
    docker compose rm -f sim_local
    current_mod_time=$(find nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
    last_mod_time="$current_mod_time"
    if [ $# -ne 0 ]; then
        docker exec -it f1tenth_gym_ros-sim_novnc-1 /bin/bash -c "./new_terminal.sh $NODE1 $NODE2"
    else
        docker exec -it f1tenth_gym_ros-sim_novnc-1 /bin/bash
    fi
    while true; do
        current_mod_time=$(find nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
        if (( $(echo "$current_mod_time > $last_mod_time" | bc -l) )); then
            echo "Change detected. Restarting session..."
            if [ $# -ne 0 ]; then
                docker exec -it f1tenth_gym_ros-sim_novnc-1 /bin/bash -c "./new_terminal.sh $NODE1 $NODE2"
            else
                docker exec -it f1tenth_gym_ros-sim_novnc-1 /bin/bash
            fi
            last_mod_time="$current_mod_time"
        fi
        sleep 1
    done
elif [ "$DISPLAY_VAL" == "local" ]; then
    echo "Simulation will be displayed on your local display"
    echo "The sim_novnc container will be closed"
    docker compose stop sim_novnc
    docker compose stop novnc
    docker compose rm -f sim_novnc
    docker compose rm -f novnc
    current_mod_time=$(find nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
    last_mod_time="$current_mod_time"
    if [ $# -ne 0 ]; then
        docker exec -it f1tenth_gym_ros-sim_local-1 /bin/bash -c "./new_terminal.sh $NODE1 $NODE2"
    else
        docker exec -it f1tenth_gym_ros-sim_local-1 /bin/bash
    fi
    while true; do
        current_mod_time=$(find nodes -type f -printf "%T@\n" | sort -n | tail -n 1)
        if (( $(echo "$current_mod_time > $last_mod_time" | bc -l) )); then
            echo "Change detected. Restarting session..."
            if [ $# -ne 0 ]; then
                docker exec -it f1tenth_gym_ros-sim_local-1 /bin/bash -c "./new_terminal.sh $NODE1 $NODE2"
            else
                docker exec -it f1tenth_gym_ros-sim_local-1 /bin/bash
            fi
            last_mod_time="$current_mod_time"
        fi
        sleep 1
    done
fi



