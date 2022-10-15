import os, sys
import numpy as np
from math import cos, sin, pi, floor
import pygame

class RPLidar_Sensor(object):
    '''Class for communicating with RPLidar rangefinder scanners : sampling time = 5seconds'''

    def __init__(self, PORT_NAME,angle_step=1,FOV=120,visualize=False,max_distance=0, IMIN=0, IMAX=50):
        '''Initilize RPLidar object for communicating with the sensor.

        Parameters
        ----------
        port : str
                Serial port name to which sensor is connected
        baudrate : int, optional
                Baudrate for serial connection (the default is 115200)
        timeout : float, optional
                Serial port connection timeout in seconds (the default is 1)
        logger : logging.Logger instance, optional
                Logger instance, if none is provided new instance is created
        '''
        self.PORT_NAME=PORT_NAME
        self.angle_step=angle_step
        self.visualize=visualize
        self.FOV=FOV
        self.max_distance=max_distance
        self.IMIN=IMIN
        self.IMAX=IMAX
        self.obj_coord_list=[]   

        # inializations
        self.init_lidar()
        if self.visualize:
            self.init_pygame()

    def init_pygame(self):
        from math import cos, sin, pi, floor
        import pygame
        os.putenv('SDL_FBDEV', '/dev/fb1')
        pygame.init()
        self.lcd = pygame.display.set_mode((320,240))
        pygame.mouse.set_visible(False)
        self.lcd.fill((0,0,0))
        pygame.display.update()

    def init_lidar(self):
        from rplidar import RPLidar
        while(True):
            try:
                print(f' \n - connecting to {self.PORT_NAME} ...')
                self.lidar = RPLidar(self.PORT_NAME)
                info = self.lidar.get_info()
                health = self.lidar.get_health()
                print(info); print(health)
                break
            except:
                 print(f' \n - Lidar connection Error to {self.PORT_NAME} ...')
                 
    def strop_lidar(self):
            print('Stoping.')
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print('Lidar stopped!!.')

    def process_lidar_data(self, data):
        points_list=[]
        if self.visualize:
            self.lcd.fill((0,0,0))
        for angle in range(360):
            distance = data[angle]
            # ignore initially ungathered data points + ignore points out of the FOV
            if distance >0 and ((angle> 90-int(self.FOV/2) and angle< 180 - int(self.FOV/2)) or self.FOV==360 ): # :#               
                self.max_distance = max([min([5000, distance]), self.max_distance])
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                point = (int(x),int(y))   
                points_list.append(point)
                if self.visualize:
                    normalize_point=(160 + int(x / self.max_distance * 119), 120 + int(y / self.max_distance * 119))
                    self.lcd.set_at(normalize_point, pygame.Color(255, 0, 0))
        if self.visualize:
            print(f' {len(points_list)} objects -> {points_list}')
            pygame.display.update()
        
        return points_list

    def get_lidar_shot(self):
        scan_data = [0]*360
        try:
                                
            #for scan in lidar_device.lidar.iter_scans():
            scan = next(self.lidar.iter_scans())    
            for (_, angle, distance) in scan:
                    scan_data[min([359, floor(angle)])] = distance
            lidar_d=self.process_lidar_data(scan_data)
            self.obj_coord_list=lidar_d
            if lidar_d!=[]:
                print(f'- Lidar max distance= {np.max(lidar_d)}')
            return lidar_d

        except KeyboardInterrupt:
                self.strop_lidar() 
                sys.exit(0)


def test_lidar():
        lidar_device = RPLidar_Sensor(PORT_NAME='/dev/ttyUSB0',FOV=140,visualize=True)

        scan_data = [0]*360
        while(True):
                lidar_device.get_lidar_shot()
                
  




if __name__ == "__main__":
        test_lidar()

