#  Highway Automated Inspection System (HAIS) node


# install the needed  packages

1. **GPS sensor:** 


2. **Camera sensor:** [more details](https://jetsonhacks.com/2022/02/02/in-practice-usb-cameras-on-jetson/)

```
roslaunch realsense2_camera rs_camera.launch
cam1: 146322076320
cam2: 140122074129

#launch multiple cameras
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=146322076320 serial_no_camera2:=140122074129
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=146322076320 serial_no_camera2:=140122074129 filters:=pointcloud


roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=146322076320
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=140122074129

#enables pointcloud on both sensors
roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=146322076320 filters:=pointcloud
roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=140122074129 filters:=pointcloud
```
  
# Run data collection 
```
$ ../ros_run.sh
```


# Acknowledgement

The proposed method used some other existing preprocessing packages which were adapted with/without modifications. The main ressources are cited as follows:
*  [source1](https://github.com/)
* 