import threading
a = 1

def thread_1():
    global a
    print(f' t1 = {a}')
    a += 1

def thread_2():
    global a
    print(f' t2 = {a}')

first_thread = threading.Thread(target = thread_1)
second_thread = threading.Thread(target = thread_2)

first_thread.start()
second_thread.start()

# #11
# # import the opencv library
# import cv2
# import numpy as np

# size=(1000, 1000)
# # define a video capture object
# vid = cv2.VideoCapture(0)

# while(True):
	
# 	# Capture the video frame
# 	# by frame
# 	ret, frame = vid.read()
# 	frame_resized = cv2.resize(frame, size) 
	
# 	# Display the resulting frame
# 	print(f' - pixels={np.unique(frame_resized)}')
# 	cv2.imshow('frame', frame_resized)
	
# 	# the 'q' button is set as the
# 	# quitting button you may use any
# 	# desired button of your choice
# 	if cv2.waitKey(1) & 0xFF == ord('q'):
# 		break

# # After the loop release the cap object
# vid.release()
# # Destroy all the windows
# cv2.destroyAllWindows()
