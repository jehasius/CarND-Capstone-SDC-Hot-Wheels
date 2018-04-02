#!/bin/bash

# see https://discussions.udacity.com/t/390138/7

catkin_make
source devel/setup.sh
roslaunch twist_controller dbw_test.launch
