import os
import sys
from lib.utils import create_new_folder

def test_camera(cam_id, disp=0):
	# import the opencv library
	print('\n --> testing the camera sensor ', cam_id )
	import cv2
	import numpy as np

	# define a video capture object
	vid = cv2.VideoCapture(cam_id)
		# Capture the video frame
		# by frame	
	while(True):
		try:
			ret, frame = vid.read()
			print('\n - image size= ', frame.shape)
			#frame = cv2.resize(frame, size) 
			filename='files/cam_' + str(cam_id) + '.jpg'
			create_new_folder(os.path.dirname(filename))
			cv2.imwrite(filename, frame)
			# Display the resulting frame
			if disp!=0:
				cv2.imshow('Camera'+str(), frame)
		except Exception as e:
			print('\n error: camera port ['+str(cam_id)+']') ; print(' Exception:', e)
			break
		except KeyboardInterrupt:
			break
		
		break
				
	# After the loop release the cap object
	vid.release()
	# Destroy all the windows
	cv2.destroyAllWindows()

if __name__ == "__main__":
	print('\n\n --- Testing sensors')
	for k in range(10):
		test_camera(k)
    
