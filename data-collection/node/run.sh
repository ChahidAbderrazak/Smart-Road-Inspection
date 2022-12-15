#!/bin/bash
# free devices
sudo lsof /dev/ttyUSB0  # Lidar
cd src

############# Data collection  ################
echo "==> Start the data collection"
python main_data_collection.py

