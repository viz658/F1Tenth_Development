#!/bin/bash
# cd f1tenth_gym_ros/
. ~/rocker_venv/bin/activate
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros --volume ./scripts:/sim_ws --volume ./nodes:/sim_ws/src/nodes -- f1tenth_gym_ros