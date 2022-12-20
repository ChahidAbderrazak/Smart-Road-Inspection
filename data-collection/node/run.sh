#!/bin/bash
############# Data collection RPI  ################
rm -r /home/pi/Desktop/upload
unzip -o /home/pi/Downloads/node.zip -d  /home/pi/Downloads/
cd /home/pi/Downloads/src

echo "==> Start the data collection [RPI]"
python main_RPI.py
