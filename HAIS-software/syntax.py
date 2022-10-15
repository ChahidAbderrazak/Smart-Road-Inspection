from nuscenes.nuscenes import NuScenes

dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/NuScience/nuScenes'
version='v1.0-mini'

dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/hais-node/2022-10-11/UOIT-parking-Abderrazak' 
version='v1.0'

dataroot='/home/abdo2020/Desktop/demo-hais-data' 
version='v1.0'

nusc_dataloder = NuScenes(version=version, dataroot=dataroot, verbose=True)

recs = [(nusc_dataloder.get('sample', record['first_sample_token'])['timestamp'], record) for record in
                nusc_dataloder.scene]

print(nusc_dataloder.list_scenes())

# # import argparse
# import imutils
# from imutils.video import FPS
# import cv2
# import matplotlib.pyplot as plt

# # define the color ranges
# # colorRanges = [
# # 	((29, 86, 6), (64, 255, 255), "green"),
# # 	((57, 68, 0), (151, 255, 255), "blue")]
# sh=90
# colorRanges = [
# 	((172-sh, 182-sh, 172-sh), (172, 182, 172), "detection")]
# video_path='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/dash-CAM/2022.08.05/VID_015.MOV'
# camera = cv2.VideoCapture(video_path)
# fps = FPS().start()

# # keep looping
# while True:
# 	# grab the current frame
# 	(grabbed, frame) = camera.read()

# 	# if we are viewing a video and we did not grab a frame, then we have
# 	# reached the end of the video
# 	if video_path and not grabbed:
# 		break

# 	# resize the frame, blur it, and convert it to the HSV color space
# 	frame = imutils.resize(frame, width=600)
# 	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
# 	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# 	# loop over the color ranges
# 	for (lower, upper, colorName) in colorRanges:
# 		# construct a mask for all colors in the current HSV range, then
# 		# perform a series of dilations and erosions to remove any small
# 		# blobs left in the mask
# 		# print(hsv)
# 		mask = cv2.inRange(hsv, lower, upper)
		
# 		mask = cv2.erode(mask, None, iterations=2)
# 		mask = cv2.dilate(mask, None, iterations=2)

# 		# find contours in the mask
# 		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# 		cnts = cnts[0]# if imutils.is_cv2() else cnts[1]

# 		# only proceed if at least one contour was found
# 		# if cnts==None:
# 		# 	cnts=[]
# 		if len(cnts) > 0:
# 			# plt.imshow(mask)
# 			# plt.show()
# 			# find the largest contour in the mask, then use it to compute
# 			# the minimum enclosing circle and centroid
# 			c = max(cnts, key=cv2.contourArea)
# 			((x, y), radius) = cv2.minEnclosingCircle(c)
# 			M = cv2.moments(c)
# 			(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

# 			# only draw the enclosing circle and text if the radious meets
# 			# a minimum size
# 			if radius > 10:
# 				cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
# 				cv2.putText(frame, colorName, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
# 					1.0, (255, 0, 0), 2)

# 	# show the frame to our screen
# 	cv2.imshow("Frame", frame)
# 	key = cv2.waitKey(1) & 0xFF

# 	# # stop the timer and display FPS information
# 	# fps.stop()
# 	# print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
# 	# print(f"[INFO] approx. FPS: {int(1/fps.elapsed())} f/s")

# 	# print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# 	fps.start()


# 	# if the 'q' key is pressed, stop the loop
# 	if key == ord("q"):
# 		break

# # cleanup the camera and close any open windows
# camera.release()
# cv2.destroyAllWindows()



	# def conactenate_all_sensors_json(self):
	# 	sensors_file_pattern=osp.join(self.config["raw_data"], 'sweeps','Json_files', '*.json')
	# 	self.list_json_files=glob(sensors_file_pattern)
	# 	print(f'\n\n #### {len(list_json_files)} data collections where found!')

	# 	table=[]
	# 	for file in self.list_json_files:
	# 		with open(file) as f:
	# 				new_tble = json.load(f)
	# 		table+=new_tble
		# # save the concatenated files
		# self.save_json(table, self.scenes_path)