# step1: roslaunch realsense2_camera rs_camera.launch  (launches realsense camera)
# step2: roslaunch rplidar_ros rplidar.launch   (launches rplidar)
# step3: rosrun hais logdata.py  (starts the data Logging process)

#!/bin/bash
clear
# gnome-terminal  --tab --title=" ROS-core" -e "sh -c 'cd ~; cd catkin_ws ; roscore'" \
#                 --tab -e "sh -c 'sleep 10; source devel/setup.bash ; roslaunch  rplidar_ros rplidar.launch '" \
#                 --tab -e "sh -c 'sleep 10; source devel/setup.bash ; roslaunch realsense2_camera rs_camera.launch '" \
#                 --tab -e "sh -c 'sleep 10; rosrun hais gps_publisher.py '" \
#                 --tab -e "sh -c 'sleep 10; rosrun hais imu_publisher.py '" \
#                 --tab -e "sh -c 'sleep 20; rosrun hais main_ros_datalogger.py '" 

gnome-terminal  --tab --title=" ROS-core" -e "sh -c 'cd ~; cd catkin_ws ; roscore'" \
                --tab -e "sh -c 'sleep 10; source devel/setup.bash ; roslaunch realsense2_camera rs_camera.launch '" \
                --tab -e "sh -c 'sleep 10; rosrun hais gps_publisher.py '" \
                --tab -e "sh -c 'sleep 10; rosrun hais imu_publisher.py '" \
                --tab -e "sh -c 'sleep 20; rosrun hais main_ros_datalogger.py '"

                # rosrun rqt_graph rqt_graph