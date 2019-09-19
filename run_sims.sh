#!/bin/bash

declare -a STARTS=(
    "start_x:=0.4   start_y:=12.2"
    "start_x:=14    start_y:=7.5"
    "start_x:=-12.8 start_y:=16.8"
    "start_x:=-9  start_y:=-2.2"
    "start_x:=8.4   start_y:=-2.2"
)
declare -a DOORS=(
    "8 4"
    "4 4"
    "-1.4 8"
    "-4 4"
)
declare -a PLANNERS=(
    "adstar"
    "fbe"
    "nbvp"
)
KNOWN_YAW=1.57079632679
SIMS_PER_CONFIG=5

for i in $(seq 1 $SIMS_PER_CONFIG)
do
    for start in "${STARTS[@]}"
    do
        for door in "${DOORS[@]}"
        do
            for planner in "${PLANNERS[@]}"
            do
                mon launch tb_simulation "$planner.launch" $start&
                launch_pid=$!
                rosrun tb_simulation sim_manager $door $KNOWN_YAW
                kill -9 $launch_pid
                sleep 0.1
            done
        done
    done
done
