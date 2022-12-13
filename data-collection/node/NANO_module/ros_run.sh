#!/bin/bash
# gnome-terminal --tab -e "sh -c 'cd catkin_ws ; roscore'" \
#                --tab -e "sh -c 'sleep 5; source devel/setup.bash ; roslaunch hector _slam_launch tutorial.launch'"