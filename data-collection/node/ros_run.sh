# step1: roslaunch realsense2_camera rs_camera.launch  (launches realsense camera)
# step2: roslaunch rplidar_ros rplidar.launch   (launches rplidar)
# step3: rosrun hais logdata.py  (starts the data Logging process)

#!/bin/bash
gnome-terminal  --tab -e "sh -c 'cd ~; cd catkin_ws ; roscore'" \
                --tab -e "sh -c 'sleep 5; source devel/setup.bash ; roslaunch  rplidar_ros rplidar.launch '" \
                --tab -e "sh -c 'sleep 5; source devel/setup.bash ; roslaunch realsense2_camera rs_camera.launch '" \
                --tab -e "sh -c 'sleep 10; rosrun hais main_ros_datalogger.py '" 
