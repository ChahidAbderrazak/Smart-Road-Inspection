import pyrebase, os, requests, eventlet, pathlib, shutil
# from urllib.parse import urlparse

firebaseConfig = {
"apiKey": "AIzaSyAOoDI0ZMIci3jOol9yLKMP7f0x5RQ34l0",
"authDomain": "recipeappdjango.firebaseapp.com",
"databaseURL": "https://recipeappdjango-default-rtdb.firebaseio.com",
"projectId": "recipeappdjango",
"storageBucket": "recipeappdjango.appspot.com",
"messagingSenderId": "439529214802",
"appId": "1:439529214802:web:ea4881111111ddd53e608b",
"measurementId": "G-V2SBLFY749",
"serviceAccount": "recipeappdjango-firebase-adminsdk-63i16-bb9afe57c7.json"
}
# Initialize Firebase HOW MANY IMAGES/SIZE OF IMAGES
firebase = pyrebase.initialize_app(firebaseConfig)
storage = firebase.storage()
db = firebase.database()

mission_status = db.child('drone_one').child('mission').child('000').child('missionstatus').get().val()

mission_imgs = storage.child('mission').child('000').child('images').list_files()
print('mission status', mission_status)

if (mission_status == 0):
	for image in mission_imgs:
		if '000/images/' in image.name:
			try:
				basename = os.path.basename(image.name)
				# print(basename)

				img = storage.child(image.name).download(basename)
				imgpath = os.path.abspath(basename)
				original = r'{}'.format(imgpath)
				target = r'images/' + basename
				shutil.move(original, target)

			except FileNotFoundError:
				print('Not Found')
				continue
			except IsADirectoryError:
				print('No Image, Skipping Directory')
				continue
			except AttributeError:
				print('Attribute Error, Skipping Directory')
				continue
		else:
			print('NOT IN 000/IMAGES')
		
	# Call meshroom_batch.exe --input images/{} --output builds/{}.format(img_folder_name, builds_folder_name)
		# Target output folder name = mission number (i.e. 000, 005, 010)

	# python listener.py input_folder output_folder mission_number

	# if mission already complete, download existing images and start meshroom

	# UI VIEW IMAGES FROM SPECIFIC MISSION
