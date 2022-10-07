import os, sys
from lib.lidar_sensor import *

def stop_lidar():
	lidar_device = RPLidar_Sensor(PORT_NAME='/dev/ttyUSB0',visualize=False)
	print('Stoping.')
	lidar_device.lidar.stop()
	lidar_device.lidar.stop_motor()


if __name__ == "__main__":
	stop_lidar()
    
