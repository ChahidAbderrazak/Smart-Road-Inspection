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

def inspection_patch_matching(img_path, hole_patch_root, patch_size=(256,256), detect_th=0.75, disp=True):
    '''
    DSP-based road inspection using patch matching
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

def evalute_damage(damage_mask_rgb, th=[0.08, 0.11]):
    from skimage.color import rgb2gray
    # evaluate the damage
    damage_mask= rgb2gray(damage_mask_rgb)    
    zeros= len(np.where(damage_mask==0)[0])

    n,m=damage_mask.shape
    all_pixels= len(np.where(damage_mask>0)[0])
    all_pixels=n*m
    if all_pixels==0:
        all_pixels=1

    deep= len(np.where(damage_mask<=th[0])[0])-zeros
    small = len(np.where(damage_mask>th[1])[0])  
    
    medium = all_pixels - deep -small - zeros
    # print(f'\n - zeros={zeros}, \n - small={small}, \n - deep={deep}, \n - all_pixels={all_pixels} \n - medium={medium}') 
    deep=int(100*deep/all_pixels)
    medium=int(100*medium/all_pixels)
    small=int(100*small/all_pixels)

    dict={  'deep':f'{deep}%',
            'medium':f'{medium}%',
            'small':f'{small}%'}
    return dict

def residual_thresholding(image0, diff_image, bright_th=160, erosion_tol=1, 
                        blur_size=(5, 5), cnt_th=[110, 255], 
                        cnt_size_ratio=0.4, disp=True):
    import imutils
    # print(f'\n - image: size= {image.size} , pixels[{np.min(image)}, {np.max(image)}]')
    image=image0.copy()
    n,m, _=diff_image.shape
    lane=diff_image.copy()
    lane[diff_image<bright_th]=0
    # lane[diff_image>=bright_th]=255
    # image erosion
    lane=image_erosion(lane, erosion_tol=erosion_tol)
    resized = imutils.resize(lane, width=300)
    ratio = lane.shape[0] / float(resized.shape[0])
    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    if blur_size==(0,0):
        blurred=gray
    else:
        blurred = cv2.GaussianBlur(gray, blur_size, 0)

    thresh = cv2.threshold(blurred, cnt_th[0], cnt_th[1], cv2.THRESH_BINARY)[1]
    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    # loop over the contours
    cimg = np.zeros_like(lane)
    nb_cnts=0
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        area = cv2.contourArea(c)
        # input(f'\n - flag: countour {area}')
        area_th=cnt_size_ratio*np.max([n,m])
        if area<area_th:
            continue
        else:
            nb_cnts+=1
            # print(f'\n - image [{n}X{m}]:  th={area_th}, area {area}')

        try:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            # box sizze
            box_sz=cX*cY
            shape = sd.detect(c)
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            # input(f'\n cX={cX} \n cY={cY} \n shape={shape} \n c={c[0]} ')
            # fitting the line
            rows,cols = image.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            # plot contours and line
            # cv2.line(lane,(cols-1,righty),(0,lefty),(0,0,255),2)
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            # cv2.putText(lane, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                # 0.5, (255, 255, 255), 2)
            # Create a mask image that contains the contour filled in
            cv2.drawContours(cimg, [c], -1, color=255, thickness=-1)
           
            
        except Exception as e:
            continue

    # final mask
    mask = np.zeros_like(image)
    mask[cimg==255]=image[cimg==255]
    mask[image>150]=0
    # evaluate the damage
    dict_damage=evalute_damage(mask)
    print(f' damage report={dict_damage}')
    # display
    if disp:
        utils.show_two_image(image, mask,img_title=f'road damages', figsize = (8,8))

    return lane, mask, nb_cnts, dict_damage

def inspection_diff(img_path, bright_th=0, erosion_tol=1, 
                    cnt_th=[5, 300], cnt_size_ratio=0.1, disp=True):
    '''
    DSP-based road inspection using image derivatives
    '''
    def image_diff(I):
        import cv2
        import numpy as np
        import matplotlib.pyplot as plt
        from scipy import ndimage

        I =  I.astype(np.float64)

        #-Derivative x
        Kx = -1*np.array([[-1,0,1]])
        Fx = ndimage.convolve(I, Kx)

        #-Derivative y
        Ky = -1*np.array([[-1],[0],[1]])
        Fy = ndimage.convolve(I, Ky)

        #--Magnitute
        magnitude = np.sqrt(Fx**2 + Fy**2) # G

        return magnitude

    print(f'\n ===>  DSP-based road inspection')
    import numpy as np
    import matplotlib.pyplot as plt
    from glob import glob
    from skimage import color
    from skimage.color import rgb2gray, gray2rgb

    # load the image
    image_RGB = utils.load_image(img_path)
    # convert to gray scale
    diff_RGB=image_RGB.copy()
    # image difference
    for k in range(3):
        diff_RGB[:,:,k]=image_diff(image_RGB[:,:,k])

    # image thresholding
    _, damage_mask, nb_cnts, dict_damage = residual_thresholding(image_RGB, diff_RGB, bright_th=bright_th, 
                                                   erosion_tol=erosion_tol, blur_size=(0, 0), cnt_th=cnt_th, 
                                                    cnt_size_ratio=cnt_size_ratio, disp=disp)
    print(f'\n - nb damagges =  {nb_cnts}')
    damage_img=cv2.addWeighted(image_RGB, 1, damage_mask, 0.8, 0)
    # utils.show_two_image(image_RGB, damage_img, img_title=f'Road inspection[{nb_cnts} damages]', figsize = (8,8))

    return damage_img, damage_mask, nb_cnts, dict_damage

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

#%%##### LANE Reflection ceoficient
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

def image_thresholding(image, bright_th=160, erosion_tol=1, 
                        blur_size=(5, 5), cnt_th=[110, 255], 
                        cnt_size_ratio=0.4):
    import imutils
    # print(f'\n - image: size= {image.size} , pixels[{np.min(image)}, {np.max(image)}]')
    n,m, _=image.shape
    lane=image.copy()
    lane[image<bright_th]=0
    # lane[image>=bright_th]=255
    # image erosion
    lane=image_erosion(lane, erosion_tol=erosion_tol)

    resized = imutils.resize(lane, width=300)
    ratio = lane.shape[0] / float(resized.shape[0])
    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    if blur_size==(0,0):
        blurred=gray
    else:
        blurred = cv2.GaussianBlur(gray, blur_size, 0)

    thresh = cv2.threshold(blurred, cnt_th[0], cnt_th[1], cv2.THRESH_BINARY)[1]
    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    # loop over the contours
    cimg = np.zeros_like(lane)
    nb_cnts=0
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        area = cv2.contourArea(c)
        # input(f'\n - flag: countour {area}')
        area_th=cnt_size_ratio*np.max([n,m])
        if area<area_th:
            continue
        else:
            nb_cnts+=1
            # print(f'\n - image [{n}X{m}]:  th={area_th}, area {area}')

        try:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            # box sizze
            box_sz=cX*cY
            shape = sd.detect(c)
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            # input(f'\n cX={cX} \n cY={cY} \n shape={shape} \n c={c[0]} ')
            # fitting the line
            rows,cols = lane.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            # plot contours and line
            # cv2.line(lane,(cols-1,righty),(0,lefty),(0,0,255),2)
            cv2.drawContours(lane, [c], -1, (0, 255, 0), 2)
            cv2.putText(lane, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 2)

            # Create a mask image that contains the contour filled in
            cv2.drawContours(cimg, [c], -1, color=255, thickness=-1)
            # Access the image pixels and create a 1D numpy array then add to list
            pts = np.where(cimg == 255)
            # utils.show_two_image(image, cimg,img_title=f'lane mark image', figsize = (8,8))
        except Exception as e:
            continue

    # final mask
    lane_mrk_mask = np.zeros_like(lane)
    lane_mrk_mask[cimg == 255]=image[cimg == 255]
    return lane, lane_mrk_mask, nb_cnts

def lane_inspection(img_path, bright_th=160, erosion_tol=1, disp=1):
    import cv2
    # Load the image
    image = cv2.imread(img_path)
    n,m, _=image.shape
    max_pixel=np.max(image)

    # image thresholding
    lane, lane_mrk_mask, nb_cnts = image_thresholding(image=image, bright_th=bright_th, erosion_tol=erosion_tol)
    # lane=np.multiply(cimg,lane)
    pixels= list(lane_mrk_mask[lane_mrk_mask>0])
    reflection_coef=(np.median(pixels)/max_pixel)
    # print(f'\n - ####\n - reflection_coef= {reflection_coef}')
    if nb_cnts==0:
        reflection_coef=0
    else:
        reflection_coef*=(20/nb_cnts)
    # print(f'\n - reflection_coef= {reflection_coef}')

    reflection_coef*=(len(pixels)/(n*m))
    # print(f'\n - reflection_coef= {reflection_coef}')
    if disp>0:
        utils.show_two_image(image, lane, img_title=f'lane mark image [reflection coefficient ={reflection_coef:.2f}]', figsize = (8,8))

    return reflection_coef, image, lane, lane_mrk_mask

#%%#########################################################################################
def main_DSP_segmentation():
    import cv2
    from glob import glob

    # img_folder='/media/abdo2020/DATA1/data/raw-dataset/data-demo/road-conditions-google/good-roads/'#hole/'#cracks/'#
    img_folder='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-12/Oshawa-roads/'

    n_segments=200
    compactness=10
    mean_th=0.3
    std_th=2.0
    nb_defect_segment=2
    img_ext='.jpg'
    ##############
    data_name=os.path.dirname(img_folder)
    out_video=f'results/dsp_road_segmentation_{data_name}.mp4'
    resize=(800,500)
    # inspect the images
    out = cv2.VideoWriter(out_video, cv2.VideoWriter_fourcc(*'MJPG'), 10, resize)
    list_images=utils.getListOfFiles(dirName=img_folder, ext=img_ext, path_pattern='')
    list_images.sort()
    print(f'\n - Found images =  {len(list_images)} images')
    for img_path in list_images:#[1:2]:#
        # Load the image
        img =cv2.imread(img_path)
        # DSP-base segmentation 
        road_mask = utils.segment_image_DSP(img, n_segments=n_segments, compactness=compactness, mean_th=mean_th, \
        std_th=std_th, nb_defect_segment=nb_defect_segment, disp=0)
        # show mask
        # road_seg=cv2.addWeighted(img, 1, road_mask, 0.8, 0)
        road_seg=road_mask

        # Display the diagnosed frame
        cv2.imshow('road segentation',road_seg)
        # save the video
        # out.write(road_seg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def main_patch_matching_inspection():
	
    detect_th=0.75
    factor=1
    Min_box_area=100
    from glob import glob
    import cv2
    # load template-image
    hole_patch_root='bin/holes-patches' 
    hole_patch_root='bin/holes-patches/test' 

    # img_folder='/media/abdo2020/DATA1/data/raw-dataset/road-conditions-google/hole/'#good-roads/'#cracks/'#
    img_folder='/media/abdo2020/DATA1/Datasets/data-demo/HAIS-data/demo-hais-data/HAIS_DATABASE-medium-speed'# ERC-parking'#   

    # list the existing images
    img_ext='.jpg'
    ##############
    data_name=os.path.dirname(img_folder)
    out_video=f'results/dsp_road_inspection_{data_name}.mp4'
    resize=(500,500)
    # inspect the images
    out = cv2.VideoWriter(out_video, cv2.VideoWriter_fourcc(*'MJPG'), 10, resize)
    list_images=utils.getListOfFiles(dirName=img_folder, ext=img_ext, path_pattern='')
    list_images.sort()
    print(f'\n - Found images =  {len(list_images)} images')
    for img_path in list_images[5:]:#:#
        # patch matching based inspection
        img,img_rgb, matching_score, mask = inspection_patch_matching(img_path, hole_patch_root, detect_th=detect_th, disp=False)

        # creat ebouding boxes
        img_box, mask_out, nb_box, label_boxes, img_class= utils.create_object_boxes(img, mask, Min_box_area=Min_box_area, factor=factor, disp=0)
        # # display the detection 
        # utils.display_detection(img_rgb, img_box, matching_score, mask_out, msg='Bouding boxes', cmap="gray")

        # Display the diagnosed frame
        cv2.imshow('DSP-based road inspection',img_box)
        # save the video
        # print(f'\n - images size=  {img_box.shape} ')
        img_box = cv2.resize(img_box, resize) 
        img_box = np.uint8(255 * img_box)
        out.write(img_box)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print('end')

def main_image_variation_inspection():

    bright_th=0
    erosion_tol=1
    cnt_th=[5, 300]
    cnt_size_ratio=0.1
    from glob import glob
    import cv2
    img_folder='/media/abdo2020/DATA1/Datasets/data-demo/HAIS-data/demo-hais-data/HAIS_DATABASE-medium-speed'
    img_folder='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31/HAIS_DATABASE-high-speed' 
    img_folder='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-12/Oshawa-roads'
    # list the existing images
    img_ext='.jpg'
    ##############
    data_name=os.path.dirname(img_folder)
    out_video=f'results/dsp_variation_road_inspection_{data_name}.mp4'
    resize=(800,500)
    # inspect the images
    out = cv2.VideoWriter(out_video, cv2.VideoWriter_fourcc(*'MJPG'), 10, resize)
    list_images=utils.getListOfFiles(dirName=img_folder, ext=img_ext, path_pattern='')
    list_images.sort()
    print(f'\n - Found images =  {len(list_images)} images')
    for img_path in list_images[5:]:#:#
        # variation based inspection
        damage_img, mask, nb_damages, dict_damage = \
            inspection_diff(img_path, bright_th=bright_th,
                            erosion_tol=erosion_tol, cnt_th=cnt_th, 
                            cnt_size_ratio=cnt_size_ratio, disp=False)
        
        # Display the diagnosed frame
        damage_img = cv2.resize(damage_img, resize) 
        n,m, _=damage_img.shape
        if nb_damages>1:
            color=(0,0,0)
            cv2.putText(damage_img, f'{nb_damages} damages: {str(dict_damage)}', 
            (int(0.01*n), int(0.1*m)), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, color, 2)
     
        # Display the diagnosed frame
        cv2.imshow('DSP-based road inspection',damage_img)
        # save the video
        # damage_img = np.uint8(255 * damage_img)
        out.write(damage_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print('end')

def main_lanemarker_inspection():
    from glob import glob
    import cv2
    # load image
    img_folder='/media/abdo2020/DATA1/Datasets/data-demo/HAIS-data/demo-lane-mark' 
    img_folder='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-12-12/road-and-mark/sweeps/RIGHT_CAMERA'
    ##############
    data_name=os.path.dirname(img_folder)
    out_video=f'results/lane_marker_{data_name}.mp4'
    resize=(500,500)

    # inspect the images
    out = cv2.VideoWriter(out_video, cv2.VideoWriter_fourcc(*'MJPG'),10, resize)
    for k, img_path in enumerate(glob(os.path.join(img_folder,'*'))):#enumerate([1:2]):#
        # DSP-based road segmentation
        reflection_coef, image, lane, lane_mrk_mask=lane_inspection(img_path, disp=0)#, bright_th=bright_th)

        # Display the diagnosed frame
        image = cv2.resize(image, resize) 
        n,m, _=image.shape
        if reflection_coef<0.2:
            color=(0,0,255)
        else:
            color=(255, 255, 255)
        cv2.putText(image, f'frame{k}:  reflection coef={reflection_coef:.2}', 
                    (int(0.01*n), int(0.1*m)), cv2.FONT_HERSHEY_SIMPLEX,
                     0.8, color, 2)
        cv2.imshow('Road reflection coefficient',image)

        # save the video
        out.write(image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # Destroy all the windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # syntax()
    ### DSP-based road inspection
    # main_patch_matching_inspection()
    main_image_variation_inspection()

    # # DSP-based road segmentation
    # main_DSP_segmentation()

    ## road lane inspection
    main_lanemarker_inspection()
