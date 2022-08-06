import os
import numpy as np
import matplotlib.pyplot as plt
try:
    from inspection_module.lib import utils
except:
    from . import utils


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
        rect = plt.Rectangle((x, y), wcoin, hcoin, edgecolor='g', facecolor='none', linewidth=3)
        ax2.add_patch(rect)


    ax3.imshow(mask)
    ax3.set_axis_off()
    ax3.set_title('matching patch score')
    # highlight matched region
    # ax3.autoscale(False)
    ax3.plot(x, y, '*', markeredgecolor='r', markerfacecolor='none', linewidth=5, markersize=30)
    plt.show()

def DSP_road_inspection(img_path, road_hole_RGB, detect_th=0.75, disp=True):
    '''
    DSP-based road inspection
    '''
    print(f'\n ===>  DSP-based road inspection')
    
    import numpy as np
    import matplotlib.pyplot as plt

    from skimage import data
    from skimage.feature import match_template
    
    detection_list=[]
    # load the image
    image_RGB = utils.load_image(img_path)
    # convert to gray scale
    from skimage import color
    image = color.rgb2gray(image_RGB)
    road_hole = color.rgb2gray(road_hole_RGB)
    hcoin, wcoin = road_hole.shape
    hcoin, wcoin = road_hole.shape
    # patch matching
    result = match_template(image, road_hole)
    ij = np.unravel_index(np.argmax(result), result.shape)
    print(f'\n \n - hole {len(ij)} coordinate: \n {ij}')
    x, y = ij[::-1]
    # update the detection
    detection_list.append([x,y, wcoin, hcoin])
    # final defect mask
    mask= np.pad(result, [((wcoin-1)//2, (hcoin-1)//2), ( (wcoin-1)//2,(hcoin-1)//2)], mode='constant')
    mask[mask<detect_th*np.max(mask)]=0
    # plot the dsp detectino results
    if disp:
        plot_dsp_detection(road_hole, image, result, detection_list)

    image_RGB=image_RGB[:mask.shape[0],:mask.shape[1],:]
    return image, image_RGB, mask

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

def DSP_segmentation(img_path, n_segments=200,compactness=10, mean_th=0.3, std_th=2.0, nb_defect_segment=2, disp=1):
    from skimage.util import img_as_float
    import cv2
    # Load the image
    img =img_as_float( cv2.imread(img_path))
    # DSP-base segmentation 
    dsp_color_mask1 = utils.segment_image_DSP(img, n_segments=n_segments, compactness=compactness, mean_th=mean_th, \
        std_th=std_th, nb_defect_segment=nb_defect_segment, disp=disp)




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

    detect_th=0.8
    from glob import glob
    import cv2
    # load template-image
    hole_patch_path='bin/holes-patches/hole1.jpg' 
    road_hole = utils.load_image(hole_patch_path)
    # load image
    img_folder='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/hole/'#good-roads/'#cracks/'#
    for img_path in glob(os.path.join(img_folder,'*')):#[1:2]:#
        # DSP-based road inspection
        img,img_rgb, mask = DSP_road_inspection(img_path, road_hole, detect_th=detect_th, disp=False)
        
        # # road mask extraction
        # road_mask=segmenting_road(img_rgb, disp=True)
        # # mask=np.multiply(road_mask,mask)

        # creat ebouding boxes
        img_box, mask_out, nb_box, label_boxes, img_class= utils.create_object_boxes(img, mask, Min_box_area=0, factor=1.2, disp=0)
        # display the detection 
        utils.display_detection(img, img_box, mask_out, msg='Bouding boxes', cmap="gray")


if __name__ == '__main__':
    # syntax()

    # # DSP-based road inspection
    main_DSP_road_inspection()

    # # DSP-based road segmentation
    # main_DSP_segmentation()
