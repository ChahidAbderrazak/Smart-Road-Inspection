# step1: roslaunch realsense2_camera rs_camera.launch  (launches realsense camera)
# step2: roslaunch rplidar_ros rplidar.launch   (launch rplidar)
# step3: rosrun hais logdata.py  (starts the data Logging process)

#!/bin/bash
clear

gnome-terminal  --tab --title=" ROS-core" -e "sh -c 'cd ~; cd catkin_ws ; roscore'" \
                --tab --title="launchRPLIDAR sensor" -e "sh -c 'sleep 10; catkin_make && source devel/setup.bash ; roslaunch  rplidar_ros rplidar.launch '" \
                --tab --title="3D camera sensor" -e "sh -c 'sleep 10; source devel/setup.bash ; roslaunch realsense2_camera rs_camera.launch '" \
                --tab --title="Side camera sensors" -e "sh -c 'sleep 10; rosrun hais cam_publisher.py '" \
                --tab --title="GPS sensor" -e "sh -c 'sleep 10; rosrun hais gps_publisher.py '" \
                --tab --title="IMU sensor" -e "sh -c 'sleep 10; rosrun hais imu_publisher.py '" \
                --tab --title="ROS graph" -e "sh -c 'sleep 20; rosrun rqt_graph rqt_graph '" \
                --tab --title="main datalogger" -e "sh -c 'sleep 25; rosrun hais main_ros_datalogger.py '" 