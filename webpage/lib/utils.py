import sys
import os
import re
import functools
import fnmatch
import numpy as np
import matplotlib.pyplot as plt


def moving_average(x, y, L, step):
    assert len(x)-- len(y), f'the input vectors are of diferet sizes: \
                            \n - x length= {len(x)} \n - y length= {len(y)} '
    x_k=[]; mean_k=[]; std_k=[]
    k=0;
    while k<len(y)-L:
        mean_k.append(np.mean(y[k:k+L]))
        std_k.append(np.std(y[k:k+L]))
        x_k.append(x[k+L//2])
        k+=step
    mean_=np.array(mean_k)
    std_=np.array(std_k)
    x2 =np.array(x_k)
    return x2, mean_, std_

def plot_with_fill(ax, x, y, L_frame=10, step= 1, color='r', alpha=0.2):
    if len(y)<50:
        L_frame, step = 3, 1
    x2, mean_, std_= moving_average(x, y, L_frame, step)
    # plot the figure
    ax.plot(x2, mean_, color+'-')
    ax.fill_between(x2, mean_ - std_, mean_ + std_, color=color, alpha=alpha)
    return ax

def find_subfolder(root, pattern=''):
    import os 
    all_ = os.listdir(root)
    folders = [k for k in all_ if os.path.isdir(k) and pattern in os.path.basename(k)]
    return folders

def find_subfiles(root, pattern='', ext=''):
    import os 
    all_ = os.listdir(root)
    files = [k for k in all_ if os.path.isfile(k) and pattern in os.path.basename(k) and k.endswith(ext)]
    # print(files)
    return files

def progresss_bar(nb_iter):
    '''
    create a progress bare display of maxim iteration loop = <nb_iter> 
    '''
    import progressbar
    bar = progressbar.ProgressBar(maxval=nb_iter, widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
    bar.start()
    return bar

#######################  MODEL SAVING/LOADING  #################
def save_model_arch(model, data_loader, model_path):
    x0 = iter(data_loader).next()
    try:
      from torchviz import make_dot
      x0 = iter(data_loader).next()
    #   print(f'x0={x0}')
      make_dot(model(x0[0]) , params=dict(list(model.named_parameters()))).render(model_path[:-4]+'_arch.png')#, format="png")
    except:
        print(model)
        with open(model_path[:-4]+'_arch.txt', "w") as external_file:
            print(model, file=external_file)
            external_file.close()

def save_trained_model(model_arch, model_path):
    import torch
    torch.save(model_arch.state_dict(), model_path)

def load_trained_model(clf_model, model_path):
    if os.path.exists(model_path):
        import torch
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        try:
            clf_model.load_state_dict(torch.load(model_path))
        except:
            
            clf_model.load_state_dict(torch.load(model_path, map_location=device))
    else:
        msg = f"\n\n - Error: The model path [{model_path}] does not exist OR in loading error!!!"
        raise Exception(msg)
    return clf_model


def load_classes(class_file):
    import json
    if os.path.exists(class_file):
        with open(class_file) as json_file:
            dict_ = json.load(json_file)
            classes = dict_['class_names']
    else:
        msg=f'\n\n Error: the class JSON file ({class_file}) does not exist!!!'
        raise Exception(msg)
    return classes

def save_classes(class_file, classes_list):
    import json
    dict_classes={k:label for k, label in enumerate(classes_list)}
    dict_={}
    dict_['class_names']=dict_classes
    # create the folder
    create_new_folder(os.path.dirname(class_file))
    # save the classe JSON file
    with open(class_file, 'w') as outfile:
        json.dump(dict_, outfile,indent=2)


def get_time_tag(type=0):
    from datetime import datetime
    today = datetime.now()
    if type==0:
        return today.strftime("__%Y-%m-%d")
    else:
        return today.strftime("__%Y-%m-%d-%Hh-%Mmin")
    
def plot_image(img, title, filename = ''):
  print(filename)
  # # import Image
  import matplotlib.pyplot as plt
  # for img, filename in zip(imgs, filenames):
  plt.axis('off')
  plt.imshow(img)
  plt.title(title + '\nfile = ' +  filename)
  plt.show()

def resize_image(src_image, size=(128,128), bg_color="white"): 
    from PIL import Image, ImageOps 
    # resize the image so the longest dimension matches our target size
    src_image.thumbnail(size, Image.ANTIALIAS )
    # Create a new square background image
    new_image = Image.new("RGB", size, bg_color)
    # Paste the resized image into the center of the square background
    new_image.paste(src_image, (int((size[0] - src_image.size[0]) / 2), int((size[1] - src_image.size[1]) / 2)))
    # return the resized image
    return new_image

def batch_sample_details(data, target):
    data_= data[0]
    target_=target.numpy()
    # check the data array
    print( 'Image type = ',type(data_) )
    print( 'Image size = ', data_.size() )
    print( 'Image min = ',data_.min() )
    print( 'Image max = ',data_.max() )
    
    for i in np.unique(target_):
        val = len(target [ target_== i] ) 
        print( '# sample in  class' + str(i) + ' = ',val )

def save_image(img, filename):
    import numpy as np
    from PIL import Image
    im = Image.fromarray(img)
    try:
        im.save(filename)
    except:
        msg = f'\n Cannot save [{filename}]. The format is unsupported!!! '
        raise Exception(msg)

def create_new_folder(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)

def create_folder_set(DIR):
  import shutil
  # Start the algo
  if not os.path.exists(DIR):
      os.makedirs(DIR)
  else:
    shutil.rmtree(DIR) 
    os.makedirs(DIR)
    print('\n Warrning: old  folder was removed and replaced by a new empthy folder!! \n', DIR)

# def copy_files(df, dst):
#   import shutil
#   create_folder_set(dst)
#   for k, path in enumerate(df['filename']): 
#     dst_path = dst + df['image_id'][k] + ext
#     # print('dst_path=', dst_path)
#     newPath = shutil.copy(path, dst_path)
#     print('\n- Copied from: %s \n- To: %s'%(path, dst_path) )

def arrange_files(root, classes, data_paths):

  data_dict = {}
  for classe in classes:
    data_dict[classe] = []
#   print('data_dict = ', data_dict) 
  sep_OS = os.path.join('1','1')[1]   
#   print('classes=', classes)
  # print('data_paths=', data_paths)
#   print('sep_OS=', sep_OS)
#   print('data_paths[0]=', data_paths[0])

  for img_path in data_paths:
    sub_folder = img_path.split(sep_OS)[0]
    # idx = classes.index(sub_folder)
    # class_folder[idx].append(img_path)
    data_dict[sub_folder] = data_dict[sub_folder] + [ img_path]

  return data_dict

def get_paths_each_class(root, ext_list):
  import os
  from glob import glob
  if root[-1] != '/':
    root = root + '/'

  L = len(root)
  data_paths = []
  img_ext = []
  for ext in  ext_list:
    result = [y[L:] for x in os.walk(root) for y in  glob(os.path.join(x[0] , "*" + ext))]
    if not result==[]:
      img_ext.append(ext)
      data_paths = data_paths + result

  assert len(data_paths)>0 ,  f' No data was found. please check these inputs : \n - directory ={root} \n - data extenstions = {ext_list}'
  classes = [ file for file in sorted(os.listdir(root)) if os.path.isdir(os.path.join(root, file))]
  data_paths_dict = arrange_files(root, classes, data_paths)

  return data_paths_dict, img_ext, classes

def save_variables(filename, var_list): 
    """
     Save stored  variables list <var_list> in <filename>:
     save_variables(filename, var_list)
    """
     
    import pickle
    open_file = open(filename, "wb"); 
    pickle.dump(var_list, open_file); 
    open_file.close()
    
def load_variables(filename):
    """
     Load stored  variables From <filename>:
     img3D_ref_prep_, img3D_faulty_prep_, mask_fault_ = load_variables(var_filename)
    """
    import pickle
    open_file = open(filename, "rb")
    loaded_obj=  pickle.load(open_file)
    open_file.close()
    return loaded_obj

def convert_tiff_save_jpg(loadFolder, file_names, saveFolder,size):
  sep_OS = os.path.join('1','1')[1] 
  if not os.path.exists(saveFolder):
    create_new_folder(saveFolder)
  # progress bar 
  bar = progresss_bar(len(file_names))
  for k,  file_name in enumerate(file_names):
    # Open the file
    file_path = os.path.join(loadFolder, file_name)
    # print("reading " + file_path)
    from PIL import Image
    image = Image.open(file_path)
    # correct the image mode
    if file_path[-4:]==".tif" or file_path[-5:]==".tiff" :
        image=image.point(lambda i:i*(1./256)).convert('L')
    # Create a resized version and save it
    try:
        resized_image = resize_image(image, size)
        file_name_tag = '_'.join(file_name.split(sep_OS))
        saveAs = os.path.join(saveFolder, file_name_tag[:-4]+'.jpg')
        # print("writing " + saveAs)
        resized_image.save(saveAs, "JPEG")
    except:
        print(f'warnning: an error was occured dutin the resizing and JPEG \
                conversion of the image: \n {file_name} ')
    #update progess bar
    bar.update(k)

def build_dataset_workspace(raw_data_folder, RAW_DATA_ROOT, ext_list, size, DIR_TRAIN, DIR_TEST, DIR_DEPLOY, dev=False, dev_size=100, split_size=0.8):
    # Display
    print('\n\n###############################################################################')
    print('#             Building the Dataset workspace from Raw data  ')
    print('#             Building the Dataset workspace from Raw data  ')
    print('###################################################################################')
  

    if raw_data_folder:
        # remove the old workspace
        DIR_WORKSPACE = os.path.dirname(os.path.dirname(DIR_TRAIN))
        create_folder_set(DIR_WORKSPACE )
        # The folder contains a subfolder for each class 
        data_paths_dict, img_ext, classes = get_paths_each_class(RAW_DATA_ROOT, ext_list)
        print(f'\n--> Data preparation information: \n - raw data folder: {RAW_DATA_ROOT} \n - destination workspace: {DIR_WORKSPACE}\n - Classes: {classes}')
        from sklearn.model_selection import train_test_split
        # Loop through each subfolder in the input folder
        for sub_folder in classes:
            print(f'\n\n--> Processing in proress of folder [{sub_folder}] ' )
            # Create a matching subfolder in the output dir
            saveFolder_train = os.path.join(DIR_TRAIN,sub_folder)
            saveFolder_test =  os.path.join(DIR_TEST,sub_folder); 
            saveFolder_deploy = DIR_DEPLOY
            # Loop through the files in the subfolder
            file_names = data_paths_dict[sub_folder]
            # print('file_names:\n', file_names)
            # print('saveFolder_train:\n', saveFolder_train)

            # get sall data set for dev purposes
            if dev:
                print(f'\n\n--> Warning!!!: Reduced size data will be generated for the development phase of class [{sub_folder}]:  \n- original data size {len(file_names)}')
                file_names = file_names[:dev_size]
                print(f'- dev size {len(file_names)}')

            # split Train/test
            files_train ,files_test0 = train_test_split(file_names, test_size=1-split_size)

            # split Train/test
            files_test ,files_deploy = train_test_split(files_test0, test_size=0.4)
            print( '\n\n--> The data is split as follows: \n- Train = %d images \n- Test = %d images  \n- Deploy = %d images  '%(len(files_train), len(files_test), len(files_deploy)))
            # save Deploy
            print('\n\n--> Data preparation: \n  -> converting/resizing/saving jpg format of the deploy set folder. Please wait ..:)\n')
            convert_tiff_save_jpg(RAW_DATA_ROOT, files_deploy, saveFolder_deploy, size)
            # save Train
            print('  -> converting/resizing/saving jpg format of the train set folder. Please wait ..:)\n')
            convert_tiff_save_jpg(RAW_DATA_ROOT, files_train, saveFolder_train, size)
            # save Test
            print('  -> converting/resizing/saving jpg format of the test set folder. Please wait ..:)\n')
            convert_tiff_save_jpg(RAW_DATA_ROOT, files_test, saveFolder_test, size)
            print('\n_________________________________________________________________________________\n')

        print('\n\n--> Data prepration is done successfully !!!.')
    else:
        print('\n\n--> The train and test set are already split.')
        classes = sorted(os.listdir(DIR_TRAIN))
        print('\n\n--> classes = ', classes)

def get_workspace_path(RAW_DATA_ROOT, WORKSPACE_folder, dev=False):
    RAW_DATA_ROOT.replace('\\','/')
    data_TAG = 'CLASS-' + '_'.join(RAW_DATA_ROOT.split('/')[-2:])
    if dev :
        data_TAG = data_TAG + '-dev'
    DIR_WORKSPACE = os.path.join(WORKSPACE_folder , data_TAG,'')
    
    return DIR_WORKSPACE, data_TAG

def get_workspace_folders(DIR_WORKSPACE):
    DIR_TRAIN = os.path.join(DIR_WORKSPACE , 'train/')
    DIR_TEST = os.path.join(DIR_WORKSPACE , 'test/')
    DIR_DEPLOY = os.path.join(DIR_WORKSPACE , 'deploy/')

    return DIR_TRAIN, DIR_TEST, DIR_DEPLOY

def get_subfolders( root, patern = ''):
        return [ name for name in os.listdir(root) if os.path.isdir(os.path.join(root, name)) if patern in name  ]
        
def find_recursive(root_dir, ext='.jpg'):
    files = []
    for root, dirnames, filenames in os.walk(root_dir):
        for filename in fnmatch.filter(filenames, '*' + ext):
            files.append(os.path.join(root, filename))
    return files


class AverageMeter(object):
    """Computes and stores the average and current value"""
    def __init__(self):
        self.initialized = False
        self.val = None
        self.avg = None
        self.sum = None
        self.count = None

    def initialize(self, val, weight):
        self.val = val
        self.avg = val
        self.sum = val * weight
        self.count = weight
        self.initialized = True

    def update(self, val, weight=1):
        if not self.initialized:
            self.initialize(val, weight)
        else:
            self.add(val, weight)

    def add(self, val, weight):
        self.val = val
        self.sum += val * weight
        self.count += weight
        self.avg = self.sum / self.count

    def value(self):
        return self.val

    def average(self):
        return self.avg


def unique(ar, return_index=False, return_inverse=False, return_counts=False):
    ar = np.asanyarray(ar).flatten()

    optional_indices = return_index or return_inverse
    optional_returns = optional_indices or return_counts

    if ar.size == 0:
        if not optional_returns:
            ret = ar
        else:
            ret = (ar,)
            if return_index:
                ret += (np.empty(0, np.bool),)
            if return_inverse:
                ret += (np.empty(0, np.bool),)
            if return_counts:
                ret += (np.empty(0, np.intp),)
        return ret
    if optional_indices:
        perm = ar.argsort(kind='mergesort' if return_index else 'quicksort')
        aux = ar[perm]
    else:
        ar.sort()
        aux = ar
    flag = np.concatenate(([True], aux[1:] != aux[:-1]))

    if not optional_returns:
        ret = aux[flag]
    else:
        ret = (aux[flag],)
        if return_index:
            ret += (perm[flag],)
        if return_inverse:
            iflag = np.cumsum(flag) - 1
            inv_idx = np.empty(ar.shape, dtype=np.intp)
            inv_idx[perm] = iflag
            ret += (inv_idx,)
        if return_counts:
            idx = np.concatenate(np.nonzero(flag) + ([ar.size],))
            ret += (np.diff(idx),)
    return ret


def colorEncode(labelmap, colors, mode='RGB'):
    labelmap = labelmap.astype('int')
    labelmap_rgb = np.zeros((labelmap.shape[0], labelmap.shape[1], 3),
                            dtype=np.uint8)
    for label in unique(labelmap):
        if label < 0:
            continue
        labelmap_rgb += (labelmap == label)[:, :, np.newaxis] * \
            np.tile(colors[label],
                    (labelmap.shape[0], labelmap.shape[1], 1))

    if mode == 'BGR':
        return labelmap_rgb[:, :, ::-1]
    else:
        return labelmap_rgb


def accuracy(preds, label):
    valid = (label >= 0)
    acc_sum = (valid * (preds == label)).sum()
    valid_sum = valid.sum()
    acc = float(acc_sum) / (valid_sum + 1e-10)
    return acc, valid_sum


def intersectionAndUnion(imPred, imLab, numClass):
    imPred = np.asarray(imPred).copy()
    imLab = np.asarray(imLab).copy()

    imPred += 1
    imLab += 1
    # Remove classes from unlabeled pixels in gt image.
    # We should not penalize detections in unlabeled portions of the image.
    imPred = imPred * (imLab > 0)

    # Compute area intersection:
    intersection = imPred * (imPred == imLab)
    (area_intersection, _) = np.histogram(
        intersection, bins=numClass, range=(1, numClass))

    # Compute area union:
    (area_pred, _) = np.histogram(imPred, bins=numClass, range=(1, numClass))
    (area_lab, _) = np.histogram(imLab, bins=numClass, range=(1, numClass))
    area_union = area_pred + area_lab - area_intersection

    return (area_intersection, area_union)


class NotSupportedCliException(Exception):
    pass


def process_range(xpu, inp):
    start, end = map(int, inp)
    if start > end:
        end, start = start, end
    return map(lambda x: '{}{}'.format(xpu, x), range(start, end+1))

REGEX = [
    (re.compile(r'^gpu(\d+)$'), lambda x: ['gpu%s' % x[0]]),
    (re.compile(r'^(\d+)$'), lambda x: ['gpu%s' % x[0]]),
    (re.compile(r'^gpu(\d+)-(?:gpu)?(\d+)$'),
     functools.partial(process_range, 'gpu')),
    (re.compile(r'^(\d+)-(\d+)$'),
     functools.partial(process_range, 'gpu')),
]

def parse_devices(input_devices):

    """Parse user's devices input str to standard format.
    e.g. [gpu0, gpu1, ...]

    """
    ret = []
    for d in input_devices.split(','):
        for regex, func in REGEX:
            m = regex.match(d.lower().strip())
            if m:
                tmp = func(m.groups())
                # prevent duplicate
                for x in tmp:
                    if x not in ret:
                        ret.append(x)
                break
        else:
            raise NotSupportedCliException(
                'Can not recognize device: "{}"'.format(d))
    return ret
