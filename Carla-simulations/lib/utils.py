import os



def create_new_folder(DIR):
	if not os.path.exists(DIR):
		os.makedirs(DIR)
