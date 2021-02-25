#!/bin/bash
# Must run proj2_init.launch in another first

# change to current ee4308 workspace
export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

# set the world and its parameters
export TURTLE_GOALS="1.9,-0.8|0.6,1.1|0.0,0.0"
export TURTLE_INF="0.2"
export TURTLE_CELL="0.1"
export TURTLE_X="0.0"
export TURTLE_Y="0.0"
export TURTLE_MAP="-10.0,-10.0,10.0,10.0"
echo '[2021PROJ1] Parameters set'

# source the workspace
source devel/setup.bash

# run the programs.
roslaunch ee4308_bringup project1_main.launch
