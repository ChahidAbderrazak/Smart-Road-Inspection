import os, sys
import numpy as np
from math import cos, sin, pi, floor

class RPLidar_Sensor(object):
    '''Class for communicating with RPLidar rangefinder scanners : sampling time = 5seconds'''

    def __init__(self, PORT_NAME,angle_step=1,repeat_lidar=20, FOV=120,visualize=False,max_distance=0, IMIN=0, IMAX=50):
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
        self.repeat_lidar=repeat_lidar
        self.FOV=FOV
        self.max_distance=max_distance
        self.IMIN=IMIN
        self.IMAX=IMAX
        self.lidar_connected=True
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

    def init_lidar(self, ):
        from rplidar import RPLidar
        cnt=0
        while(True):
            try:
                print(f' \n - connecting [{cnt}] to {self.PORT_NAME} ...')
                self.lidar = RPLidar(self.PORT_NAME)
                info = self.lidar.get_info()
                health = self.lidar.get_health()
                print(info); print(health)
                break
            except:
                 print(f' \n - Warrning: Faild attempts to connect the lidar using  port {self.PORT_NAME} ...')
            cnt+=1
            if cnt>self.repeat_lidar:
                self.lidar_connected=False
                print(f' \n - Error: The lidar is not conencted to {self.PORT_NAME} ...')
                break
                 
    def stop_lidar(self):
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
            
            if distance >0 and ((angle> 90-int(self.FOV/2) and angle< 90+int(self.FOV/2)) or self.FOV==360 ): # :#    
                self.max_distance = max([min([5000, distance]), self.max_distance])
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                point = (int(x),int(y))   
                points_list.append(point)
                if self.visualize:
                    import pygame
                    normalize_point=(160 + int(x / self.max_distance * 119), 120 + int(y / self.max_distance * 119))
                    self.lcd.set_at(normalize_point, pygame.Color(255, 0, 0))
        if self.visualize:
            import pygame
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
                self.stop_lidar() 
                sys.exit(0)



def test_lidar():
	from rplidar import RPLidar
	PORT_NAME='/dev/ttyUSB0'
	print('Stconnecting to {PORT_NAME}')
	lidar = RPLidar(PORT_NAME)

	info = lidar.get_info()
	print(info)

	health = lidar.get_health()
	print(health)

	for i, scan in enumerate(lidar.iter_scans()):
		print('%d: Got %d measurments' % (i, len(scan)))
		if i > 10:
			break

	lidar.stop()
	lidar.stop_motor()
	lidar.disconnect()

def test_vizualize_lidar():
	lidar_device = RPLidar_Sensor(PORT_NAME='/dev/ttyUSB0',visualize=True)
	scan_data = [0]*360
	try:
	
		for scan in lidar_device.lidar.iter_scans():
			for (_, angle, distance) in scan:
					scan_data[min([359, floor(angle)])] = distance
			lidar_d=lidar_device.process_lidar_data(scan_data)
			lidar_device.obj_coord_list=lidar_d
			print(str(get_time_tag(type=1)))

	except KeyboardInterrupt:
			print('Stoping.')
			lidar_device.lidar.stop()
			lidar_device.lidar.stop_motor()
			lidar_device.lidar.disconnect()
			print('Lidar stopped!!.')

if __name__ == "__main__":
        test_lidar()
        # test_vizualize_lidar()

