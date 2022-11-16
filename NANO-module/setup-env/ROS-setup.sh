#!/bin/bash

# Install ROS Melodic packages
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop-full
apt search ros-melodic
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update
printenv | grep ROS
ls ls c
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
ls /usr/bin/python*
echo $ROS_PACKAGE_PATH
sudo apt-get install -y ros-melodic-ros-tutorials
rospack find roscpp
echo $ROS_PACKAGE_PATH
roscd roscpp/cmake
roscd log
rosls roscpp_tutorials
rosls
cd
cd catkin_ws/
rosls
# create the hais-node
catkin_create_pkg hais-node std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make
rospack depends1 hais-node
roscd hais-node/
cat package.xml
source /opt/ros/melodic/setup.bash
cd ../
catkin_make
ls src/
catkin_make
# run the master node ROS
roscore
