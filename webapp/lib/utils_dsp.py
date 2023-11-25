from cProfile import label
import os
import gc
import cv2
import time
import numpy as np
import pandas as pd
from glob import glob
from pathlib import Path
from numpy.core.records import array
import matplotlib.pyplot as plt

#%% ####################   LOAD INPUT IMAGES/VOLUMES  ######################    
def check_supported_data_paths(path):
    file_type = os.path.isfile(path)
    if file_type:
        # Draw rek rek filesL
        filename, file_extension = os.path.splitext(path)
        if file_extension == '.rek' or file_extension == '.bd' or file_extension == '.nii' or file_extension == '.nrrd':
            return '3d'
        elif file_extension == '.tif' or file_extension == '.tiff' or file_extension == '.jpg' or file_extension == '.png':
            return '2d'
        else:
            msg = '\n Warning: The file format : %s is not supported!!!!'%(path)
            raise Exception(msg)

    else:
        print('Error: the input path [{path}] is not a file!!!!')
        return '-1'

def get_n_parents_folders_from_path(path, TAG='', n=2):
    '''
    get n parent folder path structure from the input path
    '''
    def get_parent(path):
        return os.path.basename(os.path.dirname(path)) 

    if n==1 or path=='':
        return  os.path.join(get_parent(path), TAG)
    else:
        n=n-1
        # if TAG=='':
        #     TAG = os.path.join(get_parent(path), TAG)
        # else:
        #     TAG = get_parent(path)
        TAG = os.path.join(get_parent(path), TAG)

        return get_n_parents_folders_from_path(os.path.dirname(path), TAG=TAG, n=n)

def get_file_tag(path):
    TAG = os.path.basename(os.path.dirname(path)) + '--' + os.path.basename(path)
    return TAG

def get_folder_tag(path):
    TAG = os.path.basename(os.path.dirname(os.path.dirname(path))) + '--' + os.path.basename(os.path.dirname(path))
    return TAG

def load_image(path):
    import numpy as np
    import cv2
    # print(' The selected image is :', path)
    filename, file_extension = os.path.splitext(path)
    try:
        img = cv2.imread(path,0)
    except:
        msg = '\n Error: the image path ' + path + 'cannot be loaded!!!!'
        raise Exception(msg)
    print(f' The selected image file is [{path}] of size {img.shape}')
    return img

def save_image(img, filename):
		import numpy as np
		import cv2
		# im = Image.fromarray(img)
		try:
			cv2.imwrite(filename, img)
			# im.save(filename)
		except:
			msg = f'\n Cannot save [{filename}]. The format is unsupported!!! '
			raise Exception(msg)


def load_data(path):
    data_type_ref = check_supported_data_paths(path)
    if data_type_ref=='3d':       
        return load_volume(path)

    elif data_type_ref=='2d':       
        return load_image(path) 

    else:
        msg = f'\n\n the input file format [{path}] is not supported !!!'
        raise Exception(msg)

def load_raw_data(path):
    if os.path.exists(path):
        if os.path.isdir(path):
            return load_slices_to_volume(path)
            
        elif os.path.isfile(path):
            return load_data(path)
    else:
        print(path)
        msg = f'\n\nError: the path [{path}] cannot be found!!!!\nplease recheck the configuration file config/config.yml\n\n'
        raise Exception(msg)

def load_input_data_path_or_array(file):
    '''
    load image/volume from the supported file format
    '''
    if isinstance(file, str):# load the input file
        return load_raw_data(file)
    if isinstance(file, np.ndarray): # copy the input array
        return file # np.copy(file)
    else:
        msg=f'\n Error: the input [ {file} ] must be a file path or data array!!!'
        raise Exception(msg)

def load_slices_to_volume(root, ext='.tiff'):
    from glob import glob 
    try : 
        os.path.isdir(root)
        list_slices_paths = glob(os.path.join(root, '*' + ext ))
    except:
        list_slices_paths = root
    
    return read_concatenate_slices_(list_slices_paths)

def read_concatenate_slices_(list_slices_paths):
    # sort the slces by name
    list_slices_paths.sort()
    from PIL import Image
    import numpy as np
    slices = []
    img_arr=[]
    # concatenate the slices (axix0) 
    # print(f'\n    -> reconstructing the reconstruction slice of {os.path.dirname(list_slices_paths[0])}')  
    for img_path in list_slices_paths:
        img = Image.open(img_path)
        slices.append(img)
        img_arr.append(np.array(img))
    #stack the 2D arrays over one another to generate the 3D array
    volume_array=np.stack(img_arr, axis=1)
    input(img_arr[0].shape)     
    return  volume_array

def single_raw2np(path, shape = (750,372), dtype=np.uint16):
	
    """

    Convert single raw image to numpy array

    """

    return np.fromfile(path,dtype=dtype,count=np.prod(shape)).reshape(shape)

def VGI2Py(file_path):
    import os
    # Using readlines()
    vgi_file =file_path + '.vgi'
    if not os.path.exists(vgi_file):
        print(' Error: The VGI file is not found: \n', vgi_file)

        return 0
    else:
        file1 = open(vgi_file, 'r')
        Lines = file1.readlines()
        count = 0
        # Strips the newline character
        for line in Lines:
            count += 1
            line_str= line.strip()
            terms = line_str.split(' ')
            # print("Line{}: {} \n {}".format(count, line_str, terms ))

            if terms[0]=='size':
                size=(int(terms[2]), int(terms[3]), int(terms[4]))
                print(' size = ', size)
            elif terms[0]=='bitsperelement':
                if terms[2]=='8':
                    voxel_type = np.uint8

                elif terms[2]=='16':
                    voxel_type = np.uint16

                else:
                    print(' Voxel type is not an usual value = ', terms[2])

            
                print(' voxel_type = ', voxel_type)

            elif terms[0]=='SkipHeader':
                SkipHeader=int(terms[2])
                print(' SkipHeader = ', SkipHeader)
            
        # load the BCM volume
        voxel_count = size[0] * size[1] * size[2]
        f = open(file_path,'rb') #only opens the file for reading
        vol_arr=np.fromfile(f,dtype=voxel_type,offset=SkipHeader,count=voxel_count)
        f.close()
        vol_arr=vol_arr.reshape(size[0],size[1],size[2])
        return vol_arr

def simpleRek2Py(filename, image_width, image_height, image_depth, voxel_datatype):
    
    '''
    filename: path to the rek file
    
    image_width x image_height x image_depth: the dimension to be resized
    500x500x500 - 0.2 GB file
    1000x1000x1000 - 1.4 GB file
    2000x2000x2000 - 15.5 GB file

    voxel_datatype: the datatype of the file
    uint16 - integer data file
    float32 - float data file
    '''
    print('\n Opening rek file: %s\n  - size=(%d,%d,%d) \n  - voxel_datatype=%s '%(filename, image_width, image_height, image_depth,voxel_datatype))
    
    if (voxel_datatype == "uint16"):
        datatype = np.uint16
    elif (voxel_datatype == "float32"):
        datatype = np.float32
    else:
        raise ValueError("Unsupported datatype")

    with open(filename, 'rb') as fd:
        raw_file_data = fd.read()        
    image = np.frombuffer(raw_file_data[2048:], dtype=datatype)
    shape = image_width, image_height, image_depth

    return image.reshape(shape)

def create_new_directory(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)

def parent_folder(file_path, n=1):
    for k in range(n):
        file_path=os.path.dirname(file_path)
        if file_path=='':
            return file_path
    return os.path.basename(file_path)

def get_extension(filename):
    return os.path.splitext(filename)[1][1:]

def getListOfFiles(dirName, ext, path_pattern='', allFiles = list()):
    '''
      For the given path, search for the List of all files in the directory tree of extention <ext> 
    '''
    from re import search
    # create a list of file and sub directories 
    # names in the given directory 
    listOfFile = os.listdir(dirName)
    # Iterate over all the entries
    for entry in listOfFile:
        # Create full path
        fullPath = os.path.join(dirName, entry)
        # If entry is a directory then get the list of files in this directory 
        if os.path.isdir(fullPath) :
            allFiles = allFiles + getListOfFiles(fullPath, ext=ext, path_pattern=path_pattern)
        else:
            # txt_list_files = [i for i in fullPath if ]
            extension = os.path.splitext(fullPath)[1][1:]
            if extension==ext and search(path_pattern, fullPath) :
              allFiles.append(fullPath) 
    # print(f'\n dirName[{ext}]= {dirName}\n allFiles={allFiles}')           
    return list(set(allFiles)) 

def getListOfFolders(dirName, ext, path_pattern='', allFolders = list()):
    '''
      For the given path, search for the List of all files in the directory tree of extention <ext> 
    '''
    from re import search
    # create a list of file and sub directories 
    # names in the given directory 
    listOfFolder = os.listdir(dirName)
    # Iterate over all the entries
    for entry in listOfFolder:
        # Create full path
        fullPath = os.path.join(dirName, entry)
        # If entry is a directory then get the list of folders in this directory 
        if os.path.isdir(fullPath):
            allFolders = allFolders + getListOfFolders(fullPath, ext=ext, path_pattern=path_pattern)
        else:
            # txt_list_folders = [i for i in fullPath if ]
            extension = os.path.splitext(fullPath)[1][1:]
            if extension==ext and search(path_pattern, fullPath):
                allFolders.append(os.path.dirname(fullPath) )
                break
    # print(f'\n dirName[{ext}]= {dirName}\n allFolders={allFolders}')           
    return list(set(allFolders)) 

def remove_ref_files(ref_scan, input_data_list):
    ref_list=[]
    file_path=[]
    for file_path in input_data_list:
        if os.path.join('',ref_scan,'') in file_path:
            input_data_list.remove(file_path)
            ref_list.append(file_path)
    # if no reference scan is found 
    if len(ref_list)==0:
        msg = f'\n\n Error: No scan named {ref_scan} id found! \
              \n Please recheck the following parameters: \
              \n - the reference scan name =  {ref_scan}'
        if len(input_data_list)==0:
            msg = msg + f'\n - data folder path: {file_path}'
        raise Exception(msg)
    # if more then one reference scan are found 
    if len(ref_list)>1:
        print(f'\n warning: More than one scan named {ref_scan} are found:  \
              \n {ref_list} . \n ny the first is considered = {ref_list[0]}')
    
    return ref_list[0]

def save_array_to_2rgb(arr, filename, scale=255):
    '''
    save a 2D array to jpg image in RGB format
    arr : input 2D numpy array
    filename : ipg image file name
    scale : save image in a different scale. Default [0, 255]
    '''
    from PIL import Image
    if len(arr.shape) == 2:
        create_new_directory(os.path.dirname(filename) )
        img_rgb = img_array_gray2rgb(arr, scale=scale)
        img = Image.fromarray(img_rgb)
        img.save(filename, "JPEG")
    else:
        msg = f'\n\n--> Error: the input array dimension [{arr.shape}] is not a 2D image!!!'
        raise Exception(msg)

def img_array_gray2rgb(array_gray, scale=255):
    # convert grey image, to 3-channel RGB
    img_ = np.copy(array_gray)
    del array_gray
    gc.collect()
    # normaliztion and scaling
    if scale > 1:
        if np.max(img_)!=0:
            img_= img_ /np.max(img_)
            img_ = img_*scale
    img_rgb = np.zeros((*img_.shape,3), dtype=np.uint8)
    for k in range(3):
        img_rgb[:,:,k]=img_
    return img_rgb

#