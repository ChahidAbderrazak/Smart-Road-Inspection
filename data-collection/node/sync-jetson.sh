#!/bin/bash

clear
################### [Jetson-remote] : compress the codes ###################
echo && echo && echo "--> Compressing the ROS scripts"
cd /home/hais/catkin_ws/src; tar -cf /home/hais/catkin_ws/Jetson_ROS_src.tar *

############### [local host] : Import code from the jetson #################
echo && echo && echo "-->  Importing the scripts to the host"
rsync -azP  hais@192.168.0.111:/home/hais/catkin_ws/Jetson_ROS_src.tar    /home/chahid/Desktop/code