#!/bin/bash

clear
mkdir /home/hais/Desktop/workspace

################### [Jetson-remote] : compress the codes ###################
echo && echo && echo "--> Compressing the ROS scripts"
cd /home/hais/catkin_ws/src; tar -cf /home/hais/Desktop/workspace/Jetson_ROS_src.tar *
cd /home/hais/catkin_ws/src/hais/src; tar -cf /home/hais/Desktop/node.tar *

echo && echo && echo "--> Compressing the collected dataset"
cd /home/hais/Desktop/upload; tar -cf /home/hais/Desktop/workspace/Jetson_ROS_data.tar *
############### [local host] : Import code from the jetson #################
echo && echo && echo "-->  Importing the scripts to the host"
rsync -azP  hais@192.168.0.111:/home/hais/Desktop/workspace/    /home/chahid/Desktop/code