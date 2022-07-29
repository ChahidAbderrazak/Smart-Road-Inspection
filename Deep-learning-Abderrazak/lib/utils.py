
import os
import cv2
import time
import gc
import numpy as np
import pandas as pd
from glob import glob
import matplotlib.pyplot as plt

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

def display_detection(img, img_box, mask,  msg='', cmap="cividis"):
    print(f'\n\n --> 2D inspection results.\n- image size ={img_box.shape}')
    if len(img_box.shape)==3 and img_box.shape[2]!=3:
        import random 
        idx = random.randint(1,img_box.shape[0])
        img_box = img_box[idx]
    # display
    fig, (ax3, ax4) = plt.subplots(1, 2, sharex=True, sharey=True, figsize=(30, 8))
    ax3.imshow(img_box, interpolation='none', cmap=cmap)
    ax3.set_title('Road damage localization ')
    ax4.imshow(img, cmap='gray', interpolation='none', origin='lower')
    ax4.imshow(mask, cmap='Reds', interpolation='none', alpha=0.6)
    ax4.set_title(f'Road damage  segmentation ')
    plt.show()


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
                                               (int(prop.bbox[3]*factor), int(prop.bbox[2]*factor)), (255, 0, 0), 4)
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




#%%######################   DSP BASED ROAD INSPECTION  ######################

def plot_dsp_detection(road_hole, image, mask, detection_list):
    fig = plt.figure(figsize=(30, 8))
    ax1 = plt.subplot(1, 3, 1)
    ax2 = plt.subplot(1, 3, 2)
    ax3 = plt.subplot(1, 3, 3)#, sharex=ax2, sharey=ax2)

    ax1.imshow(road_hole, cmap=plt.cm.gray)
    ax1.set_axis_off()
    ax1.set_title('road hole template')

    ax2.imshow(image, cmap=plt.cm.gray)
    ax2.set_axis_off()
    ax2.set_title('Image image')
    ax2.set_xlabel('Dtected Hole (Green)')
    

    # highlight matched region
    for x,y, wcoin, hcoin in detection_list:
        rect = plt.Rectangle((x, y), wcoin, hcoin, edgecolor='g', facecolor='none', linewidth=5)
        ax2.add_patch(rect)


    ax3.imshow(mask)
    ax3.set_axis_off()
    ax3.set_title('matching patch score')
    # highlight matched region
    # ax3.autoscale(False)
    ax3.plot(x, y, '*', markeredgecolor='r', markerfacecolor='none', linewidth=5, markersize=30)
    plt.show()

def DSP_road_inspection(img_path, road_hole, disp=True):
    '''
    DSP-based road inspection
    '''
    print(f'\n ===>  DSP-based road inspection')
    
    import numpy as np
    import matplotlib.pyplot as plt

    from skimage import data
    from skimage.feature import match_template
    hcoin, wcoin = road_hole.shape
    detection_list=[]
    # load the image
    image = load_image(img_path)
    result = match_template(image, road_hole)
    result[result<0.75*np.max(result)]=0
    ij = np.unravel_index(np.argmax(result), result.shape)
    print(f'\n \n - hole {len(ij)} coordinate: \n {ij}')
    x, y = ij[::-1]
    # update the detection
    detection_list.append([x,y, wcoin, hcoin])
    # final defect mask
    mask= np.pad(result, [((wcoin-1)//2, (hcoin-1)//2), ( (wcoin-1)//2,(hcoin-1)//2)], mode='constant')
    # plot the dsp detectino results
    if disp:
        plot_dsp_detection(road_hole, image, result, detection_list)

    return image, mask
