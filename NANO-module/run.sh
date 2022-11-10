#!/bin/bash
###############  TEST SENSORS  ###############

# #------------------- ENV + IMH Sensors  --------------- Yes
# cd data_collection/lib/ENV_sensors
# echo "# Testing the Kinetic/weather sensors"
# python3 ENV_IMU_sensors.py

# #------------------- CAMERA sensor --------------- Yes
# echo "# Testing the CAMERA sensors"
# python3 data_collection/lib/camera.py


# #------------------- DUAL CAMERA sensor --------------- NO
# cd  data_collection/lib
# echo "# Testing the DUAL CAMERA sensors"
# python3 dual_camera.py

# #------------------- GPS sensor --------------- 1/2
# cd  data_collection/lib/SIM7600X_4G_for_JETSON_NANO/GPS
# echo "# Testing the GPS sensors"
# python3 GPS.py


#------------------- LIDAR sensor --------------- NO
echo "# Testing the Lidar sensors"
python3 data_collection/lib/lidar_sensor.py



# ##############  DATA COLLECTION ##############
# cd  data_collection
# echo "# Start the data collection"
# python start_data_collection_serial.py

