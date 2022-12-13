import os
import glob

def getListOfFiles(dirName, ext, path_pattern='', allFiles0 = []):
		'''
		For the given path, search for the List of all files in the directory tree of extention <ext> 
		'''
		from re import search
		# create a list of file and sub directories 
		# names in the given directory 
		listOfFile = os.listdir(dirName)

		if allFiles0==[]:
			allFiles=[]
		else:
			allFiles=allFiles0.copy()


		if ext[0]=='.':
				ext=ext[1:]
		# Iterate over all the entries
		for entry in listOfFile:
				# Create full path
				fullPath = os.path.join(dirName, entry)
				# If entry is a directory then get the list of files in this directory 
				if os.path.isdir(fullPath) :
						allFiles = allFiles + getListOfFiles(fullPath, ext=ext, path_pattern=path_pattern)

				elif search(path_pattern, fullPath) :
								extension = os.path.splitext(fullPath)[1][1:]
								if extension==ext:
										allFiles.append(fullPath) 
				# print(f'\n dirName[{ext}]= {dirName}\n allFiles={allFiles}')					 
		allFiles=[k for k in allFiles if ext in os.path.splitext(k)[1][1:] == ext]
		return list(set(allFiles)) 

