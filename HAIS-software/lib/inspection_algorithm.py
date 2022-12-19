import os
import numpy as np
import matplotlib.pyplot as plt
try:
    from lib import utils
except:
    import utils


#%%######################   DSP BASED ROAD INSPECTION  ######################

def plot_dsp_detection(road_hole, image, matching_score, mask, detection_list):
    fig = plt.figure(figsize=(30, 8))
    ax1 = plt.subplot(1, 4, 1)
    ax2 = plt.subplot(1, 4, 2)
    ax3 = plt.subplot(1, 4, 3)#, sharex=ax2, sharey=ax2)
    ax4 = plt.subplot(1, 4, 4)#, sharex=ax2, sharey=ax2)

    ax1.imshow(road_hole, cmap=plt.cm.gray)
    ax1.set_axis_off()
    ax1.set_title('road hole template')

    ax2.imshow(image, cmap=plt.cm.gray)
    ax2.set_axis_off()
    ax2.set_title('Image image')
    ax2.set_xlabel('Dtected Hole (Green)')
    
    ax3.imshow(matching_score, cmap=plt.cm.gray)
    ax3.set_axis_off()
    ax3.set_title('matching score')
    

    # highlight matched region
    for x,y, wcoin, hcoin in detection_list:
        rect = plt.Rectangle((x, y), wcoin, hcoin, edgecolor='g', facecolor='none', linewidth=3)
        ax3.add_patch(rect)


    ax4.imshow(mask)
    ax4.set_axis_off()
    ax4.set_title('matching patch score')
    # highlight matched region
    # ax4.autoscale(False)
    ax4.plot(x, y, '*', markeredgecolor='r', markerfacecolor='none', linewidth=5, markersize=30)
    
    
    plt.show()

def detect_road_damage_using_patch_matching(image_RGB, road_hole_RGB, disp=False):
    # convert to gray scale
    from skimage import color
    from skimage import data
    from skimage.feature import match_template
    image = color.rgb2gray(image_RGB)
    road_hole = color.rgb2gray(road_hole_RGB)
    hcoin, wcoin = road_hole.shape
    hcoin, wcoin = road_hole.shape
    # patch matching
    result = match_template(image, road_hole)
    mask= np.pad(result, [((wcoin-1)//2, (hcoin-1)//2), ( (wcoin-1)//2,(hcoin-1)//2)], mode='constant')
    ij = np.unravel_index(np.argmax(result), result.shape)
    # print(f'\n \n - hole {len(ij)} coordinate: \n {ij}')
    x, y = ij[::-1]
    detection=[x,y, wcoin, hcoin]
    # # plot the dsp detectino results
    # if disp:
    #     plot_dsp_detection(road_hole_RGB, image, result, [detection])

    return  mask,result, detection

def resize_image(src_image, size): 
    if isinstance(src_image, np.ndarray):
        from PIL import Image
        src_image =  Image.fromarray(src_image)
        src_image = src_image.point(lambda i:i*1).convert('L')
    # resize the image so the longest dimension matches our target size
    src_image.thumbnail(size, Image.ANTIALIAS )
    # Create a new square background image
    new_image = Image.new("RGB", size)
    # Paste the resized image into the center of the square background
    new_image.paste(src_image, (int((size[0] - src_image.size[0]) / 2), int((size[1] - src_image.size[1]) / 2)))
    # return the resized image
    return new_image

def DSP_road_inspection(img_path, hole_patch_root, patch_size=(256,256), detect_th=0.75, disp=True):
    '''
    DSP-based road inspection
    '''
    print(f'\n ===>  DSP-based road inspection')
    import numpy as np
    import matplotlib.pyplot as plt
    from glob import glob
    from skimage import color

    patch_paths=glob(os.path.join(hole_patch_root,'*'))
    detection_list=[]
    # load the image
    image_RGB = utils.load_image(img_path)

    # deblur the image
    image_RGB=deblur_image(image_RGB)

    # resize the images
    # size=(128,128)
    # image_RGB=resize_image(image_RGB, size)

    # convert to gray scale
    image_RGB = np.array(image_RGB)
    image = color.rgb2gray(image_RGB)
    # mask= 0*np.empty_like( image)
    # result= 0*np.empty_like( image)
    # print(f'flag: seg_image={np.unique(image_RGB)}')

    # patchs matching
    for patch_path in patch_paths:
        road_hole_RGB = utils.load_image(patch_path)
        # input(f'flag: seg_hole={np.unique(road_hole_RGB)}, size={len(np.unique(road_hole_RGB))}')
        if len(np.unique(road_hole_RGB))==1:
            continue
        # run the detection
        mask_, result_, detection = detect_road_damage_using_patch_matching(image_RGB, road_hole_RGB)
        # update the detection
        detection_list.append(detection)
        try:
            mask+=mask_
            matching_score+=result_
        except:
            mask=mask_
            matching_score=result_
    # final defect mask
    mask[mask<detect_th*np.max(mask)]=0
    image_RGB=image_RGB[:mask.shape[0],:mask.shape[1],:]
    # plot the dsp detectino results
    if disp:
        plot_dsp_detection(road_hole_RGB, image, matching_score, mask, detection_list)
    return image, image_RGB, matching_score, mask

def segmenting_road(img, disp=False):
    from skimage import data, segmentation, color
    from skimage.future import graph
    from matplotlib import pyplot as plt

    labels1 = segmentation.slic(img, compactness=10, n_segments=2000, start_label=1)
    out1 = color.label2rgb(labels1, img, kind='avg', bg_label=0)

    g = graph.rag_mean_color(img, labels1, mode='distance', connectivity=5)
    labels2 = graph.cut_threshold(labels1, g, 10)
    out2 = color.label2rgb(labels2, img, kind='avg', bg_label=0)
    seg = color.rgb2gray(out2)
    N,M=seg.shape
    xmin,xmax = int(0.5*N), int(0.85*N)
    ymin, ymax=int(0.40*M), int(0.60*M)
    pavmt=seg[xmin:xmax, ymin:ymax]

    dict_count, frequent=utils.count_pixels(pavmt)
    # get the road segment 
    road_mask=0*np.empty_like(seg)
    road_mask[seg==frequent]=1

    # print(f'\n - img size={img.shape} - road_mask size={road_mask.shape}  ')
    # print(f'\n pavmt: \n - size={pavmt.shape} \n counting= \n{dict_count} \n - frequent={frequent}')

    
    if disp:
        fig, ax = plt.subplots(nrows=3, sharex=False, sharey=False,
                        figsize=(6, 8))
        ax[0].imshow(out1)
        ax[1].imshow(seg)
        ax[2].imshow(road_mask)
        for a in ax:
            a.axis('off')

        plt.tight_layout()
        plt.show()

    return road_mask


def deblur_image(img):
    
    alpha=1
    deblur_img = alpha*vertical_mb + (1-alpha)*horizonal_mb
    return deblur_img

def DSP_segmentation(img_path, n_segments=200,compactness=10, mean_th=0.3, std_th=2.0, nb_defect_segment=2, disp=1):
    from skimage.util import img_as_float
    import cv2
    # Load the image
    img =img_as_float( cv2.imread(img_path))
    # DSP-base segmentation 
    dsp_color_mask1 = utils.segment_image_DSP(img, n_segments=n_segments, compactness=compactness, mean_th=mean_th, \
        std_th=std_th, nb_defect_segment=nb_defect_segment, disp=disp)


##### LANE Reflection ceoficient
import cv2
class ShapeDetector:
    def __init__(self):
        pass
    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"
        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"
        # return the name of the shape
        return shape

def image_erosion(img, erosion_tol=5):
    from skimage.color import rgb2gray, gray2rgb

    img_gray=rgb2gray(img)
    from scipy import ndimage
    img_filter=ndimage.binary_erosion(img_gray, structure=np.ones((erosion_tol,erosion_tol))).astype(float)
    for k in range(3):
        img[:,:,k]=np.multiply(img[:,:,k],img_filter)
    return img

def lane_inspection(img_path, bright_th=190, erosion_tol=5):
    import cv2
    import imutils
    # Load the image
    image = cv2.imread(img_path)
    max_pixel=np.max(image)
    # print(f'\n - image: size= {image.size} , pixels[{np.min(image)}, {np.max(image)}]')
    lane=image.copy()
    lane[image<bright_th]=0
    # lane[image>=bright_th]=255
    lane=image_erosion(lane, erosion_tol=erosion_tol)

    resized = imutils.resize(lane, width=300)
    ratio = lane.shape[0] / float(resized.shape[0])
    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    
    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        # input(f'\n - flag: {M}')
        try:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = sd.detect(c)
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(lane, [c], -1, (0, 255, 0), 2)
            cv2.putText(lane, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 2)
        except Exception as e:
            continue
    pixels= list(lane[lane>0])
    n,m, _=lane.shape
    reflextion_coef=10*(len(pixels)/(n*m))*(np.mean(pixels)/max_pixel)

    utils.show_two_image(image, lane,img_title=f'lane mark image [reflection ={reflextion_coef:.1f}]', figsize = (8,8))
#########################################################################################


def main_DSP_segmentation():
    
    from glob import glob
    import cv2
    # load template-image
    templ_img_path='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/hole/download (2).jpeg'
    road_hole = utils.load_image(templ_img_path)[80:120, 80:120]
    # load image
    img_folder='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/good-roads/'#hole/'#cracks/'#
    for img_path in glob(os.path.join(img_folder,'*')):#[1:2]:#
        # DSP-based road segmentation
        utils.DSP_segmentation(img_path, n_segments=200,compactness=10, mean_th=0.3, std_th=2.0, nb_defect_segment=2)

def main_DSP_road_inspection():

    detect_th=0.95
    factor=1
    Min_box_area=100
    from glob import glob
    import cv2
    # load template-image
    hole_patch_root='bin/holes-patches' 
    # hole_patch_root='bin/holes-patches/test' 

    # img_folder='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/hole/'#good-roads/'#cracks/'#
    img_folder='/media/abdo2020/DATA1/Datasets/data-demo/HAIS-data/demo-hais-data/HAIS_DATABASE-medium-speed'# ERC-parking'#   

    # list the existing images
    img_ext='.jpg'
    list_images=utils.getListOfFiles(dirName=img_folder, ext=img_ext, path_pattern='')
    list_images.sort()
    print(f'\n - Found images =  {len(list_images)} images')
    for img_path in list_images[0:1]:#
        # DSP-based road inspection
        img,img_rgb, matching_score, mask = DSP_road_inspection(img_path, hole_patch_root, detect_th=detect_th, disp=False)

        # try:
        #     img,img_rgb, matching_score, mask = DSP_road_inspection(img_path, hole_patch_root, detect_th=detect_th, disp=False)
        # except:
        #     continue
        # # road mask extraction
        # road_mask=segmenting_road(img_rgb, disp=True)
        # # mask=np.multiply(road_mask,mask)

        # creat ebouding boxes
        img_box, mask_out, nb_box, label_boxes, img_class= utils.create_object_boxes(img, mask, Min_box_area=Min_box_area, factor=factor, disp=0)
        # display the detection 
        utils.display_detection(img_rgb, img_box, matching_score, mask_out, msg='Bouding boxes', cmap="gray")

def main_lane_inspeection():
    
    from glob import glob
    import cv2
    bright_th=200
    # load image
    img_folder='/media/abdo2020/DATA1/Datasets/data-demo/HAIS-data/demo-lane-mark' 
    for img_path in glob(os.path.join(img_folder,'*')):#[1:2]:#
        # DSP-based road segmentation
        lane_inspection(img_path)#, bright_th=bright_th)



if __name__ == '__main__':
    # syntax()

    # # DSP-based road inspection
    # main_DSP_road_inspection()

    # # DSP-based road segmentation
    # main_DSP_segmentation()

    # road lane inspection
    main_lane_inspeection()



# %%
