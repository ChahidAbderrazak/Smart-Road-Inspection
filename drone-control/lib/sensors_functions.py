import os 

def get_RGB_Camera():
	frame, err = 0, 0 
	return frame, err

##### DRONE



def set_drone_control(u):
	'''
	u = [0, 0, 1]
	'''


	drone.move_left(u[0])
	drone.move_right(u[1])
	drone.rotate_clockwise(u[2])

	# import  drone functions
	data, err = 0, 0 
	return data, err

def get_drone_data():
	# import  drone functions
	data, err = 0, 0 
	return data, err