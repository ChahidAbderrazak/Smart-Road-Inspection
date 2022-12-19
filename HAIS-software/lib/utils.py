
import os
import cv2
import time
import gc
import numpy as np
import pandas as pd
from glob import glob
import matplotlib.pyplot as plt

#################	 BASIC FUNCTIONS	#################
def load_json(filename):
	try:
		if os.path.exists(filename):
			import json
			f = open(filename,)
			data = json.load(f)
			f.close()
		else:
			data=[]
		return data
	except:
		msg = f'\n\n Error: The JSON file <{filename}> cannot be read correctly!!'
		print(msg)
		# raise ValueError(msg)
		return []

def save_json(json_string, filename):
	import json
	try:
		# Using a JSON string
		with open(filename, 'w') as outfile:
			json.dump(json_string, outfile, indent=2)
			return 0
	except:
		print(f'\n\n - error in saving {filename}')
		return 1

def update_json(json_string, filename):
    old_list = load_json(filename)
    combined_list= old_list + json_string
    save_json(combined_list, filename)


def pkl_to_dict(filename):
	import pickle
	open_file = open(filename, "rb")
	dict = pickle.load(open_file)
	open_file.close()
	return dict

def get_sensor_filename(sensor_name, frame, config):
	try:
		scene_setup = '__' + config['Scenario']+ '-' + config['USE_CASE']
	except:
		scene_setup = '__Unknown'
	time_tag =  str(get_time_tag(type=1))
	return time_tag + scene_setup + '__' + sensor_name + '__' + str(frame)

def get_time_tag(type=1):
	from datetime import datetime
	today = datetime.now()
	if type==0:
			return today.strftime("__%Y-%m-%d")
	else:
			return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

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
    # filename, file_extension = os.path.splitext(path)
    try:
        img = cv2.imread(path)
    except:
        msg = '\n Error: the image path ' + path + 'cannot be loaded!!!!'
        raise Exception(msg)
    # print(f' The selected image file is [{path}] of size {img.shape}')
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

def normalize_image(img):
    img = img - img.min()
    img= img / img.max()
    return img

def load_tif_as_jpg(img_path, scale=0.0, save=False):
    from PIL import Image
    cnt = 0
    target_name ='data/temp.jpg'
    if os.path.isfile(img_path):
        img = Image.open(img_path)
        img=img.point(lambda i:i*(1./256)).convert('L')
    else:
        # convert grey image, to 3-channel RGB
        result = img_array_gray2rgb(img_path, scale=scale)
        # Save result
        img = Image.fromarray(result)
    create_new_directory(os.path.dirname(target_name))
    img.save(target_name, "JPEG")

    return  cv2.imread(target_name)


#%%######################   2D VISUALIZATIONS   ######################
def show_image(img, img_title, cmap="cividis", figsize = (8,8)):
    # show image
    fig = plt.figure(figsize = figsize) # create a 5 x 5 figure 
    ax3 = fig.add_subplot(111)
    ax3.imshow(img, interpolation='none', cmap=cmap)
    ax3.set_title(img_title)#, fontsize=40)
    # plt.savefig('./residual_image.jpg')   
    plt.axis("off")
    plt.show()

def show_two_image(img0, img1, img_title, cmap="cividis", figsize = (8,8)):
    # show image
    fig = plt.figure(figsize = figsize) # create a 5 x 5 figure 
    ax1 = fig.add_subplot(121)
    ax1.imshow(img0, interpolation='none', cmap=cmap)
    ax1.set_title(img_title)#, fontsize=40)
    
    ax2 = fig.add_subplot(122)
    ax2.imshow(img1, interpolation='none', cmap=cmap)
    

    # plt.savefig('./residual_image.jpg')   
    plt.axis("off")
    plt.show()

def show_input_images(img_input, img_ouput, msg='', cmap='gray'):
    img_ouput_ = load_image(img_ouput)
    img_input_ = load_image(img_input)
    # display
    fig, (ax1, ax2) = plt.subplots(1, 2, sharex=True, sharey=True)
    ax1.imshow(img_input_, interpolation='none', cmap=cmap)
    ax1.set_title('input image')
    ax1.set_ylabel(msg)
    ax2.imshow(img_ouput_, interpolation='none', cmap=cmap)
    ax2.set_title('output image')
    plt.show()

def display_detection(img, img_box, matching_score, mask,  msg='', cmap="cividis"):
    ij = np.unravel_index(np.argmax(matching_score), matching_score.shape)
    x, y = ij[::-1]
    print(f'\n\n --> 2D inspection results.\n- image size ={img_box.shape}')
    if len(img_box.shape)==3 and img_box.shape[2]!=3:
        import random 
        idx = random.randint(1,img_box.shape[0])
        img_box = img_box[idx]
    # display
    fig, (ax1,ax2, ax3) = plt.subplots(1, 3, sharex=True, sharey=True, figsize=(30, 8))

    ax1.imshow(img)
    ax1.set_title('CAM')

    # ax1.imshow(matching_score)
    # ax1.set_title('Pixel matching score ')
    # ax1.plot(x, y, '*', markeredgecolor='r', markerfacecolor='none', linewidth=10, markersize=30)
    ax1.set_axis_off()

    ax2.imshow(img_box)#, interpolation='none', cmap=cmap)
    ax2.set_title('Road damage localization ')
    ax2.set_axis_off()


    ax3.imshow(img, cmap='gray', interpolation='none', origin='lower')
    ax3.imshow(mask, cmap='Reds', interpolation='none', alpha=0.6)
    ax3.set_title(f'Road damage  segmentation ')
    ax3.set_axis_off()
    plt.show()

def create_new_directory(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)

def create_new_folder(DIR):
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

def getListOfFiles(dirName, ext, path_pattern='', allFiles = []):
    '''
      For the given path, search for the List of all files in the directory tree of extention <ext> 
    '''
    from re import search
    # create a list of file and sub directories 
    # names in the given directory 
    listOfFile = os.listdir(dirName)
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

######################   READ/WRITE/eXTRACT VARIABLESS   ######################
def split_filename(filename):
    '''
    get dir, file prefix, extention from the input filenames
    '''
    dir=os.path.dirname(filename)
    file_prefix, extension = os.path.splitext(os.path.basename(filename) )
    return dir, file_prefix, extension

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

def save_array_to_gray(arr, filename, scale=255):
    '''
    save a 2D array to png grayscale image 
    arr : input 2D numpy array
    filename : input image file name
    scale : save image in a different scale. Default [0, 255]
    '''
    if len(arr.shape) == 2:
        create_new_directory(os.path.dirname(filename) )
        image_int = np.array(255*arr, np.uint8)
        cv2.imwrite(filename, image_int)
    else:
        msg = f'\n\n--> Error: the input array dimension [{arr.shape}] is not a 2D image!!!'

import os
import xml.etree.cElementTree as ET
from PIL import Image

def create_file(root_dir, file_prefix, xml_filename,  width, height, voc_labels):
    root = create_root(root_dir, file_prefix, width, height)
    root = create_object_annotation(root, voc_labels)
    tree = ET.ElementTree(root)
    
    # print('xml_filename=', xml_filename)
    tree.write(xml_filename)
    
def create_root(root_dir, file_prefix, width, height, ext='.jpg'):
    root = ET.Element("annotations")
    ET.SubElement(root, "filename").text = file_prefix + ext
    ET.SubElement(root, "folder").text = root_dir
    size = ET.SubElement(root, "size")
    ET.SubElement(size, "width").text = str(width)
    ET.SubElement(size, "height").text = str(height)
    ET.SubElement(size, "depth").text = "3"
    return root

def create_object_annotation(root, voc_labels):
    for voc_label in voc_labels:
        obj = ET.SubElement(root, "object")
        ET.SubElement(obj, "name").text = voc_label[0]
        bbox = ET.SubElement(obj, "bndbox")
        ET.SubElement(bbox, "xmin").text = str(voc_label[1])
        ET.SubElement(bbox, "ymin").text = str(voc_label[2])
        ET.SubElement(bbox, "xmax").text = str(voc_label[3])
        ET.SubElement(bbox, "ymax").text = str(voc_label[4])
    return root

#%%######################### labels from detections  ######################
def get_box_coordinate(image_path, detection_boxes, w, h, defect_class, xml_list=[], disp=0):
    '''
    get labels/masks/boxes coordinates from detections 
    '''
    import os
    if disp>=1:
        print("\n\n\n ######\n Image = %s \ndetection = %s"%(image_path, detection_boxes) )
    # get file ID / prefix
    file_prefix, file_extension = os.path.splitext(os.path.basename(image_path) ) 
    voc_labels = []
    for pascal_voc_box, defect_name in zip(detection_boxes, defect_class):	
        voc_labels.append([defect_name] + list(pascal_voc_box) )
        if disp>=1:
            print('defect_name :', defect_name)
            print('pascal_voc_box :', pascal_voc_box)
        yolo_box = pascal_voc_to_yolo(pascal_voc_box[0], pascal_voc_box[1], pascal_voc_box[2], pascal_voc_box[3], w, h)
        column_name = ['filename', 'image_id', 'width', 'height', 'bbox', 'source']
        row=[image_path, file_prefix, w, h, yolo_box, defect_name]
        xml_list.append(row)
    # copy the image
    image_path = image_path.replace('\\', '/')
    file_path = image_path.replace('images/', 'annotations/')
    dst_xml_dir = os.path.dirname(file_path); create_new_directory(dst_xml_dir)
    xml_filename = os.path.join(dst_xml_dir, file_prefix + '.xml')
    create_file(dst_xml_dir, file_prefix, xml_filename, w, h, voc_labels)
    # report
    if disp>=1:
        print(f"\n\nThe annotation of image is complete! \n - image file: {image_path}  \n - xml file: {xml_filename}")

    return xml_list

def save_xml_mask_annotations(img, mask, data_tag, annotation_directory,  label_boxes, img_class, with_masks=True,  \
    CLASS_MAPPING = ['defect-free', 'defective'], defect_label ='', img_ext = '.jpg', mask_ext = '.png', disp=0):
    '''
    Save annotation : bouding boxes in xml/csv files and save the binay mask images
    Min_box_area : the minimal detectable boxes area.
    annotation_directory: directory where annotation will be saved in
    img: annotated images
    mask: annotation mask
    label_boxes: bouding boxes
    img_class: image class
    with_masks: create masks or not. False) only images for classification, True) mask for segmentation and object detection
    CLASS_MAPPING: list of all classes of this image. Default =  ['defect-free', 'defective']
    defect_label: list defining the label of each box in  <label_boxes> [ missing screw, missing spring, ...] , default:''

    '''

    if with_masks == False:
        # adjust the the destination folder
        annotation_directory=os.path.join(annotation_directory, img_class, os.path.dirname(data_tag) )
        data_tag = os.path.basename(data_tag)
        # save images and mask in RGB files
        if disp >=1:
            print(f'\n ---> saving the annotation into the folder [{annotation_directory}]')
        image_path = os.path.join( annotation_directory, data_tag + img_ext)
        save_array_to_2rgb(img, image_path)
        msg_out=  '\n- The {img_class} images are saved into the folder [{annotation_directory}]' 
        if disp >=2:
            print(msg_out)
        return msg_out

    else:
        # initializations
        CSV_DATA_FILE = os.path.join(annotation_directory, 'data.csv')
        xml_list = []
        # adjust the the destination folder
        annotation_directory=os.path.join(annotation_directory, img_class, os.path.dirname(data_tag) )
        data_tag = os.path.basename(data_tag)

        # save images and mask in RGB files
        if disp >=1:
            print(f'\n ---> saving the annotation into the folder [{annotation_directory}]')
        image_path = os.path.join( annotation_directory, 'images', data_tag + img_ext)
        save_array_to_2rgb(img, image_path)
        mask_path= os.path.join( annotation_directory, 'masks', data_tag  + mask_ext)  
        save_array_to_gray(mask, mask_path)
        # transform the inspection boxes to xml annotations
        w, h = img.shape[1], img.shape[0]   # get the image dimensions
        if  defect_label=='':            # Assign the default name of defectes as  CLASS_MAPPING[1]->'defective'                                     
            defect_label = [CLASS_MAPPING[1] for defect in label_boxes] 
        xml_list = get_box_coordinate(image_path, label_boxes, w, h, defect_label, xml_list=xml_list)
        # Save  all annotation in csv file
        column_name = ['filename', 'image_id', 'width', 'height', 'bbox', 'inspection']
        if not os.path.exists(CSV_DATA_FILE):
            xml_df = pd.DataFrame(xml_list, columns=column_name)
        else:
            xml_old = pd.read_csv(CSV_DATA_FILE)
            xml_new = pd.DataFrame(xml_list, columns=column_name)
            xml_df = pd.concat([xml_old, xml_new])

        # save the annotation
        xml_df.to_csv(CSV_DATA_FILE,index=None)
        if disp >=1:
            print('\n xml_list=', xml_list)
            print('\n CSV file is saved in : %s'%(CSV_DATA_FILE))
            print('xml_df', xml_df )

        # save classes in jason files
        import json
        DIR_CLASS = os.path.join( os.path.dirname(CSV_DATA_FILE), 'classe.json')
        with open(DIR_CLASS, 'w') as fp:
            json.dump(DIR_CLASS, fp)
        DIR_CLASS_info = os.path.join( os.path.dirname(CSV_DATA_FILE), 'info.json')   
        Class_dict = {"source": 'generated  using Hybrid inspection ', "class_names":{ str(k):name for k, name in enumerate(CLASS_MAPPING)}}
        with open(DIR_CLASS_info, 'w') as fp:
            json.dump(Class_dict, fp)
        # display
        msg_out=  f'\n- The annotation are saved into the folder [{annotation_directory}]' 
        if disp >=2:
            print(msg_out)
        return msg_out

def create_object_boxes(img_inpt0, mask, Min_box_area=0, factor=1.2, with_masks=True, data_tag='', annotation_directory='', disp=1):
    '''
    Create bouding boxes and segmentation masks for each detected fault
    img_inpt: input image
    mask: input mask
    Min_object_area : Minimal box area. smaller boxes will be ignored
    with_masks: create masks or not. False) only images for classification, True) mask for segmentation and object detection
    data_tag : a tag of the experiment/scan,etc
    annotation_directory: directory where the annoation will be save. 
                        if annotation_directory='', annotation will not be saved 
    '''

    img_inpt=np.copy(img_inpt0)
    max_img_area=img_inpt.shape[0]*img_inpt.shape[1]
    min_box_zoom_area = 0.005*max_img_area
    if disp >=1:
        print(f'\n--> creating bouding boxes using Min box area = {Min_box_area}')
    if disp >=2:
        print(f'np.sum(mask)={np.sum(mask)} ')
        print(f'np.sum(img_inpt)={np.sum(img_inpt)} ')
        print(f'img_inpt type is {type(img_inpt)} ')
        print(f'img_inpt.shape={img_inpt.shape} ')
        print(f'Min_object_area={Min_box_area} ')
    
    if img_inpt.max()<=0: #image is black/zeros
        mask_out, nb_box, label_boxes, img_class = mask, 0, [], 'defect-free'
        return img_inpt, mask_out, nb_box, label_boxes, img_class

    else:#image/mask are not black/zeros
        from skimage.measure import label, regionprops
        import cv2
        img_0 = img_inpt.copy()
        mask_0= mask.copy(); 
        mask_out=0.0*mask.copy(); 
        mask_0[mask>0]  = 1
        lbl_0 = label(mask_0, background=0, connectivity=2) 
        props = regionprops(lbl_0)
        img_box = img_0.copy(); img_box=250*(img_box/img_box.max())
        if disp>=3:
            display_detection(img_inpt, img_box, mask, msg=' input to create boxes', cmap="gray")
        try:
            img_box = cv2.cvtColor(img_box,cv2.COLOR_GRAY2RGB)
        except:
            img_box = img_array_gray2rgb(img_box)
        # Count boxes in the mask
        nb_box = 0
        label_boxes = []
        if disp>=3:
            display_detection(img_inpt, img_box, mask, msg=' input to create boxes [gray scale]', cmap="gray")
        if np.max(img_box)!=0:
            img_box= img_box / np.max(img_box)
            mask_area = mask.shape[0] * mask.shape[1]
            defect_area =  np.count_nonzero(mask)
            if disp>=1:
                print(f'\n  - mask area = {mask_area}')
                print(f'\n  - found box countors ={len(props)}')
            # count the  number of detected boxes above the Threshold
            if len(props)< mask_area/2 :
                for prop in props:
                    box_obj= [prop.bbox[1], prop.bbox[0], prop.bbox[3], prop.bbox[2]]
                    box_area = (box_obj[2]-box_obj[0]) * (box_obj[3]-box_obj[1])
                    if disp>=3:
                        print(f'\nprop={prop}')
                        print(f'box_area={box_area}')
                        print(f'Min_box_area={Min_box_area}')
                        print(f'mask shape={mask.shape}')
                        print(f'defect_area ={defect_area}')
                        print(f'\n\n box_area ={box_area}, min_box_zoom_area ={min_box_zoom_area}')

                    if box_area > Min_box_area:
                        label_boxes.append(box_obj)
                        if disp>=2:
                            print('Found bbox', prop.bbox)

                        if box_area >min_box_zoom_area:  
                            factor=1
                        cv2.rectangle(img_box, (int(prop.bbox[1]//factor), int(prop.bbox[0]//factor)), \
                                               (int(prop.bbox[3]*factor), int(prop.bbox[2]*factor)), (255, 0, 0), thickness=2)
                        nb_box+=1
                        # update th  mask
                        mask_out[prop.bbox[0]:prop.bbox[2],prop.bbox[1]:prop.bbox[3]] = mask[prop.bbox[0]:prop.bbox[2],prop.bbox[1]:prop.bbox[3]] 
                        if disp>=3:
                            display_detection(img_inpt, img_box, mask_out, msg='Bouding boxes', cmap="gray")
            msg_org = ( int(0.1*mask.shape[1]) , int(0.95*mask.shape[0]) )
            if nb_box == 0:
                # print('\n\n\n Warnning: No fault is detected !!!!')
                img_box =cv2.putText(img=np.copy(img_box), text="Good road!", org=msg_org,fontFace=1, fontScale=1, color=(0,255,0), thickness=1)
                img_class = 'defect-free'
            else:
                img_box =cv2.putText(img=np.copy(img_box), text= str(nb_box) + " damage detected!", org=msg_org,fontFace=1, fontScale=1, color=(255,0,0), thickness=1)
                img_class = 'defective'
            # save the annotaled/labeled data
            out=''
            if annotation_directory!='':
                data_tag = data_tag +f'_box{Min_box_area}'
                out = save_xml_mask_annotations(img_inpt, mask_0, data_tag, annotation_directory,  label_boxes, img_class,  \
                                        CLASS_MAPPING = ['defect-free', 'defective'], defect_label ='', with_masks=with_masks, disp=0)
            # display 
            if disp>=2:
                msg_out= '- label_boxes=' + str(label_boxes) + \
                    '\n- nb_box=' + str(nb_box) + \
                    '\n- label_boxes=' + str(label_boxes) + out
                print(msg_out)
            return img_box, mask_out, nb_box, label_boxes, img_class
            
        else:
            print(f'boxes creation is ignored as the <img_box> is black (max value= {np.max(img_box)} )')
            img_class = 'defect-free'
            return img_inpt, mask_out, nb_box, label_boxes, img_class

#%%######################   BOUNDING BOXES CONVERTIONS   ######################

# source:: https://christianbernecker.medium.com/convert-bounding-boxes-from-coco-to-pascal-voc-to-yolo-and-back-660dc6178742
# Converting Coco 
def coco_to_pascal_voc(x1, y1, w, h):
    return [x1,y1, x1 + w, y1 + h]

def coco_to_yolo(x1, y1, w, h, image_w, image_h):
    return [((2*x1 + w)/(2*image_w)) , ((2*y1 + h)/(2*image_h)), w/image_w, h/image_h]

# Converting Pascal_voc
def pascal_voc_to_coco(x1, y1, x2, y2):
    return [x1,y1, x2 - x1, y2 - y1]

def pascal_voc_to_yolo(x1, y1, x2, y2, image_w, image_h):
    return [((x2 + x1)/(2*image_w)), ((y2 + y1)//(2*image_h)), (x2 - x1)/image_w, (y2 - y1)/image_h]

# Converting Yolo 
def yolo_to_coco(x_center, y_center, w, h,  image_w, image_h):
    w = w * image_w
    h = h * image_h
    x1 = ((2 * x_center * image_w) - w)/2
    y1 = ((2 * y_center * image_h) - h)/2
    return [x1, y1, w, h]

def yolo_to_pascal_voc(x_center, y_center, w, h,  image_w, image_h):
    w = w * image_w
    h = h * image_h
    x1 = ((2 * x_center * image_w) - w)/2
    y1 = ((2 * y_center * image_h) - h)/2
    x2 = x1 + w
    y2 = y1 + h
    return [x1, y1, x2, y2]

def plot_cv_bbox(img, voc_box):
    msg_org = ( int(0.1*img.shape[1]) , int(0.95*img.shape[0]) )
    img_box =cv2.putText(img=np.copy(img), text="bbox: " + str(voc_box), \
             org=msg_org,fontFace=3, fontScale=1, color=(255,0,0), thickness=2)
    cv2.rectangle(img_box, (voc_box[1], voc_box[0]), (voc_box[3], voc_box[2]), (255, 0, 0), 4)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

############ Segmentation from input image texture [DSP-segmentation]  ######################
from skimage.filters import sobel
from skimage.color import rgb2gray, gray2rgb
from skimage.segmentation import felzenszwalb, slic, quickshift, watershed
from skimage.segmentation import mark_boundaries
from skimage.util import img_as_float
import cv2 

def create_circular_mask(h, w, center=None, radius=None):
    # https://stackoverflow.com/questions/44865023/how-can-i-create-a-circular-mask-for-a-numpy-array
    if center is None: # use the middle of the image
        center = (int(w/2), int(h/2))
    if radius is None: # use the smallest distance between the center and image walls
        radius = min(center[0], center[1], w-center[0], h-center[1])

    Y, X = np.ogrid[:h, :w]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)

    mask = dist_from_center <= radius
    return mask

def relax_mask(mask):
    masked_img = mask.copy()
    coords = np.argwhere(mask)
    try:
        x_min, y_min = coords.min(axis=0)
        x_max, y_max = coords.max(axis=0)
        cropped = mask[x_min:x_max+1, y_min:y_max+1]
        cropped = cropped + np.flipud(cropped) + np.fliplr(cropped)
        # circuler mask 
        h, w = cropped.shape[:2]
        mask_ = create_circular_mask(h, w)
        masked_circle = cropped.copy()
        masked_circle[~mask_] = 0
        masked_circle[mask_] = 1
        masked_img[x_min:x_max+1, y_min:y_max+1] = masked_circle
        return masked_img
    except:
        print(f'\n warning: mask relaxation is skipped')
        return masked_img

def  mask_relaxation_cercle(mask_union, th=1):
        mask_union[mask_union>th] = 1
        #relax the mask to avoid removing parts from the tool
        mask_union = relax_mask(mask_union)
        return mask_union

def background_removel(img, bs_level=1, disp=1):
    '''
    2D background subtruction/removal 
    bs_level : backgound subtruction: 0) disabled 1) outer-backround removal 2) all background removal 
    '''
    if bs_level==1 :
        mask = compute_outer_mask(img, disp=disp)
    elif bs_level==2:
        if len(img.shape)==3: 
            img_gray=rgb2gray(img)
        else:
            img_gray = img
        # create a binary thresholded image
        _, binary = cv2.threshold(img_gray, 0.5*img_gray.max(), img_gray.max(), cv2.THRESH_BINARY_INV)
        # masks further processing
        mask = mask = invert_binary_mask(binary)
    else:
        mask = 1+np.copy(img)
    
    # sompute the masked image
    img_=np.empty_like(img)
    if len(img.shape)==3:
        img_= np.multiply(img, gray2rgb(mask) )
    else:
        img_= np.multiply(img, mask )
    return img_, mask

def invert_binary_mask(arr):
    '''
    invert binary mask such that  1 <--> 0. 
    arr: input binary 0/1 array 
    '''
    val=np.sort(np.unique(arr))
    if len(val)==2:
        where_0 = np.where(arr == val[0])
        where_1 = np.where(arr == val[1])
        arr[where_0] = 1
        arr[where_1] = 0
    else:
        msg=f'Error: the input mask is not a binary mask. it contain {len(val)} different values {val}:  '
        print(msg)
        return arr
        # raise Exception(msg)
    return arr

def backgroung_statistics(mask, img_gray):
    idx_bckgnd = np.where(mask == 0)
    mean_bckgnd = [np.mean(img_gray[idx_bckgnd])]
    std_bckgnd = [np.std(img_gray[idx_bckgnd])]
    return mean_bckgnd, std_bckgnd

def compute_outer_mask(img_data, CANNY_THRESH_1 = 10, CANNY_THRESH_2 = 200, MASK_DILATE_ITER = 10, MASK_ERODE_ITER = 10, disp=1):
    '''
    Compute the outer mask using edges detection of the object in the image
    img_data: the input image
    CANNY_THRESH_1 : the thresold 1 of edge detection  using canny algorithm. default= 10
    CANNY_THRESH_2 : the thresold 2 of edge detection  using canny algorithm. default= 200 [np.max(img_data)+1]
    MASK_DILATE_ITER : Smoothing the mask default= 10 
    MASK_ERODE_ITER : eroding parameter. default= 10
    disp=1
    '''
    # == https://stackoverflow.com/questions/29313667/how-do-i-remove-the-background-from-this-kind-of-image
    
    import cv2
    import numpy as np
    from matplotlib import pyplot as plt
    #== load the data =======================================================================
    img = load_tif_as_jpg(img_data, scale=200)
    CANNY_THRESH_1 = 10
    CANNY_THRESH_2 = np.max(img)+1
    #== Processing =======================================================================
    # img_path = 'data/M3.jpg'#'data/person.jpg'
    # img = cv2.imread(img_path)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #-- Edge detection -------------------------------------------------------------------
    edges = cv2.Canny(gray, CANNY_THRESH_1, CANNY_THRESH_2)
    edges = cv2.dilate(edges, None)
    edges = cv2.erode(edges, None)
    #-- Find contours in edges, sort by area ---------------------------------------------
    contour_info = []
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        contour_info.append((
            c,
            cv2.isContourConvex(c),
            cv2.contourArea(c),
        ))
    contour_info = sorted(contour_info, key=lambda c: c[2], reverse=True)
    max_contour = contour_info[0]

    #-- Create empty mask, draw filled polygon on it corresponding to largest contour ----
    # Mask is black, polygon is white
    mask = np.zeros(edges.shape)
    cv2.fillConvexPoly(mask, max_contour[0], (255))

    #-- Smooth mask, then blur it --------------------------------------------------------
    mask = cv2.dilate(mask, None, iterations=MASK_DILATE_ITER)
    mask = cv2.erode(mask, None, iterations=MASK_ERODE_ITER)

    if disp>=2:
        print('img= ', img.shape)
        print('max pixel = ' , img.max())
        print('edges= ', edges.shape)
        show_input_images(img, edges, msg='edges')
        show_input_images(edges, mask, msg='mask')
    return mask

def search_similar_segment(input_mean , input_std, mean_, std_, mean_th=0.2, std_th=0.2):
    seg_new = -1
    dm = np.inf; ds = np.inf
    for cnt, (m, s) in enumerate(zip(mean_, std_)):
        if m*(1-mean_th) <= input_mean <= m*(1+mean_th) :
            if std_th==0.0 or  input_std <= std_th*s : #  input_std**2 <= (s**2)*(1+std_th) :# (s**2)*(1-std_th) <= 
                if np.abs(input_std-m) < dm and np.abs(input_std-s) < ds:
                    seg_new = cnt
                    dm = np.abs(input_std-m)
                    ds = np.abs(input_std-s)

    # if no similar region is found
    if seg_new == -1:
        seg_new = len(mean_)
        mean_.append(input_mean)
        std_.append(input_std)

    return seg_new, mean_, std_

def refine_segment_statistic(segments, img, mean_th=0.2, std_th=0.2, nb_defect_segment=1, mean_=[], std_=[]):
    print(f'\n--> Segments refininment : background statistics= mean={mean_}, std={std_}')
    list_segments= np.unique(segments)
    # print('list_segments=', list_segments)
    segments_slic_refined=np.empty_like(segments)
    for seg in list_segments:
        idx = np.where(segments == seg)
        vals= img[idx][:]
        mean , st = vals.mean(), vals.std()
        # print(f'segment{seg}, mean={mean}, std={st}')
        seg_new, mean_, std_ = search_similar_segment(mean , st, mean_, std_, mean_th, std_th)

        # assign new refined segment 
        segments_slic_refined[idx] = seg_new

    # get the defect 
    detected_defect=np.empty_like(segments_slic_refined)
    segments_label = np.unique(segments_slic_refined)
    list_means = np.sort(mean_)
    # input(f'FLAG: \n list_means={list_means} \n segments_label={segments_label} \n mean_={mean_} \n nb_defect_segment = {nb_defect_segment}')
    idx=np.where( mean_ <=list_means[nb_defect_segment-1])

    defect_list = segments_label[idx]
    for defect in defect_list:
        if defect!=0:
            idx_defect = np.where(segments_slic_refined==defect)
            detected_defect[idx_defect]=1.0#defect

    return segments_slic_refined, detected_defect

def segment_image_DSP(img0, n_segments=1000, compactness=10, mean_th=0.3, std_th=0.9, nb_defect_segment=3, disp=1):
    # Adaptive Equalization
    from skimage import exposure
    img = exposure.equalize_adapthist(img0, clip_limit=0.03)
    # Convert the image to grayscale
    img_gray=rgb2gray(img)
    # background removal
    img_, mask = background_removel(img, bs_level=2, disp=disp)

    mean_bckgnd, std_bckgnd = backgroung_statistics(mask, img_gray)

    # fig, ax = plt.subplots(2, 2, figsize=(10, 10), sharex=True, sharey=True)
    # ax[0, 0].imshow(np.multiply(img_gray, mask1 ))
    # ax[0, 0].set_title('image*mask')
    # ax[0, 1].imshow(mask1)
    # ax[0, 1].set_title('mask')
    # ax[1, 0].imshow(np.multiply(img_gray, mask ), cmap='jet')
    # ax[1, 0].set_title('image* outer-mask')
    # ax[1, 1].imshow(mask)#mark_boundaries(img, segments_slic_refined))
    # ax[1, 1].set_title('outer-mask ')

    ##https://scikit-image.org/docs/dev/auto_examples/segmentation/plot_segmentations.html
    # segments_slic = slic(img, n_segments=n_segments, compactness=compactness, sigma=0.5, start_label=1) 

    segments_slic_1 = slic(img, n_segments=n_segments, compactness=compactness, sigma=0.5, start_label=1) 

    segments_slic_2 = np.multiply(segments_slic_1, mask)

    segments_slic_refined, detected_defect = refine_segment_statistic(segments_slic_2, img_gray, mean_th=mean_th, std_th=std_th, \
                                                                       nb_defect_segment=nb_defect_segment, mean_=mean_bckgnd, std_=std_bckgnd)
    # display
    if disp>=1:
        print(f'img max={img.max()}, gray img max={img_gray.max()}')
        # Figure of comparison 
        fig, ax = plt.subplots(2, 2, figsize=(10, 10), sharex=True, sharey=True)
        ax[0, 0].imshow(mark_boundaries(img, segments_slic_1))
        ax[0, 0].set_title(f'SLIC segment Boundaries [{len(np.unique(segments_slic_1))} segments]')
        ax[1, 0].imshow(img)#mark_boundaries(img, segments_slic_refined))
        ax[1, 0].set_title('Original image')
        ax[0, 1].imshow(segments_slic_refined, cmap='hot')
        ax[0, 1].set_title(f'proposed SLIC segments refining [{len(np.unique(segments_slic_refined))} segments]')
        ax[1, 1].imshow(img, interpolation='none', origin='lower')
        # ax[1, 1].imshow(segments_slic_refined, cmap='hot', interpolation='none', alpha=0.4)
        ax[1, 1].imshow(detected_defect, cmap='Reds', interpolation='none', alpha=0.4)
        ax[1, 1].set_title(f'Defect detection [defect_th = {nb_defect_segment} segments] ')

        for a in ax.ravel():
            a.set_axis_off()

        plt.tight_layout()
        plt.show()

    return segments_slic_refined

def count_pixels(image):
    """
    Returns a count of pixels per a unique color
    Args:
        filename (str): the image to count the number of pixels of
    Returns:
        a key-value pairing of the rgb color value and the number of times the color was present in the image
    """
    color_count = {}
    width, height = image.shape
    # iterate through each pixel in the image and keep a count per unique color
    frequent=0
    max_count=0
    for x in range(width):
        for y in range(height):
            rgb = image[x, y]
            if rgb in color_count:
                color_count[rgb] += 1
            else:
                color_count[rgb] = 1
            # check the max
            if max_count<color_count[rgb]:
                frequent=rgb
                max_count=color_count[rgb]

    return color_count, frequent
    
