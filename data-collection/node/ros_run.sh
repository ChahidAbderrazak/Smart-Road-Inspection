# step1: roslaunch realsense2_camera rs_camera.launch  (launches realsense camera)
# step2: roslaunch rplidar_ros rplidar.launch   (launch rplidar)
# step3: rosrun hais logdata.py  (starts the data Logging process)

#!/bin/bash
clear
sudo chmod 666 /dev/ttyTHS1

# gnome-terminal  --tab --title=" ROS-core" -e "sh -c 'cd ~; cd catkin_ws ; roscore'" \
#                 --tab --title="launchRPLIDAR sensor" -e "sh -c 'sleep 10; catkin_make && source devel/setup.bash ; roslaunch  rplidar_ros rplidar.launch '" \
#                 --tab --title="3D camera sensor" -e "sh -c 'sleep 12; source devel/setup.bash ; roslaunch realsense2_camera rs_camera.launch '" \
#                 --tab --title="Right camera sensor" -e "sh -c 'sleep 10; rosrun hais cam_right_publisher.py '" \
#                 --tab --title="Left camera sensor" -e "sh -c 'sleep 10; rosrun hais cam_left_publisher.py '" \
#                 --tab --title="GPS sensor" -e "sh -c 'sleep 10; rosrun hais gps_publisher.py '" \
#                 --tab --title="IMU sensor" -e "sh -c 'sleep 10; rosrun hais imu_publisher.py '" \
#                 --tab --title="ROS graph" -e "sh -c 'sleep 20; rosrun rqt_graph rqt_graph '" \
#                 --tab --title="main datalogger" -e "sh -c 'sleep 25; rosrun hais main_ros_datalogger.py '" 
                
                # 


# gnome-terminal  --tab --title=" ROS-core" -e "sh -c 'cd ~; cd catkin_ws ; roscore'" \
#                 --tab --title="launchRPLIDAR sensor" -e "sh -c 'sleep 10; catkin_make && source devel/setup.bash ; roslaunch  rplidar_ros rplidar.launch '" \
#                 --tab --title="RPLidar subscriber" -e "sh -c 'sleep 15; rosrun hais lidar_subscriber.py '" \
#                 --tab --title="IMU sensor" -e "sh -c 'sleep 10; rosrun hais imu_publisher.py '" \
#                 --tab --title="GPS sensor" -e "sh -c 'sleep 10; rosrun hais gps_publisher.py '" \
#                 --tab --title="main datalogger" -e "sh -c 'sleep 10; rosrun hais main_ros_datalogger.py '" \
#                 --tab --title="3D camera publisher" -e "sh -c 'sleep 10; source devel/setup.bash ; roslaunch realsense2_camera rs_camera.launch '" \
#                 --tab --title="3D camera subscriber" -e "sh -c 'sleep 15; rosrun hais cam_3D_subscriber.py '" \
#                 --tab --title="Right camera sensor" -e "sh -c 'sleep 15; rosrun hais cam_right_publisher.py '" \
#                 --tab --title="Left camera sensor" -e "sh -c 'sleep 15; rosrun hais cam_left_publisher.py '" \
#                 --tab --title="ROS graph" -e "sh -c 'sleep 20; rosrun rqt_graph rqt_graph '" 

gnome-terminal  --tab --title=" ROS-core" -e "sh -c 'cd ~; cd catkin_ws ; roscore'" \
                --tab --title="IMU sensor" -e "sh -c 'sleep 10; rosrun hais imu_publisher.py '" \
                --tab --title="GPS sensor" -e "sh -c 'sleep 10; rosrun hais gps_publisher.py '" \
                --tab --title="main datalogger" -e "sh -c 'sleep 10; rosrun hais main_ros_datalogger.py '" \
                --tab --title="launchRPLIDAR sensor" -e "sh -c 'sleep 10; catkin_make && source devel/setup.bash ; roslaunch  rplidar_ros rplidar.launch '" \
                --tab --title="RPLidar subscriber" -e "sh -c 'sleep 15; rosrun hais lidar_subscriber.py '" \
                --tab --title="3D camera publisher" -e "sh -c 'sleep 10; source devel/setup.bash ; roslaunch realsense2_camera rs_camera.launch '" \
                --tab --title="3D camera subscriber" -e "sh -c 'sleep 15; rosrun hais cam_3D_subscriber.py '" \
                --tab --title="Left camera sensor" -e "sh -c 'sleep 15; rosrun hais cam_left_publisher.py '" \
                --tab --title="ROS graph" -e "sh -c 'sleep 20; rosrun rqt_graph rqt_graph '" 
