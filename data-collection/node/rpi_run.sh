#!/bin/bash
# free devices
sudo lsof /dev/ttyUSB0  # Lidar
cd src

############# Data collection RPI  ################
echo && echo && echo "==> Start the data collection [RPI]"
python main_RPI.py

# ############# Data collection Jetson  ################
# echo "==> Start the data collection [Jetson]"
# python main_jetson.py


