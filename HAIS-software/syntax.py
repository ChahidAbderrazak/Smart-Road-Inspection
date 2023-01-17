	
	cnt=0
	files_list=[]
	while(True):
		# filename, car_position=DB.get_file_path(sensor_name, n=1)
		filename, car_position=DB.get_file_path_ego(sensor_name, n=1)
		print(f'\n filename={filename}')
		files_list.append(filename)
		cnt+=1
		if filename=='':
			break
	print(f'\n cnt={cnt} \n unique file = {len(np.unique(files_list))}')