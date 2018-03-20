#!/usr/bin/env sh

catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
