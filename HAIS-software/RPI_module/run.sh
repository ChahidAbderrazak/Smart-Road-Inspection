#!/bin/bash
cd data_collection
echo "# Start the data collection"
python start_data_collection_serial.py

echo "# Stop the Lidar"
python stop_lidar.py
