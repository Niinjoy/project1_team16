#!/bin/bash
# change to current ee4308 workspace
export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

# assign permissions
source devel/setup.bash
chmod +x src/ee4308/ee4308_bringup/scripts/*.py
chmod +x src/ee4308/ee4308_bringup/scripts/*.sh
chmod +x src/ee4308/ee4308_turtle/scripts/*.py

# bring up rviz
roslaunch ee4308_bringup project1_init.launch
