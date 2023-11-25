import os
import numpy as np
import pandas as pd
from argparse import ArgumentParser
# Import PyTorch libraries
import torch
import torchvision.transforms as transforms
# Import the model architecture
try:
    from lib.networks import get_model_instance
    from lib.utils_dsp import *
except:
    from src.lib.networks import get_model_instance
    from src.lib.utils_dsp import *

# instanciate the model
trained_model = None

#Functions
def getListOfFiles(dirName,ext_list=['.tif', '.tiff', '.jpg']):
    # create a list of file and sub directories 
    # names in the given directory 
    listOfFile = os.listdir(dirName)
    allFiles = list()
    # Iterate over all the entries
    for entry in listOfFile:
        # Create full path
        fullPath = os.path.join(dirName, entry)
        # If entry is a directory then get the list of files in this directory 
        if os.path.isdir(fullPath):
            allFiles = allFiles + getListOfFiles(fullPath)
        else:
            allFiles.append(fullPath)
    for path in allFiles:
        ext = '.' + path.split(".")[-1] 
        if  ext not in ext_list:
            allFiles.remove(path)

    return allFiles

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

def create_new_folder(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)

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

def predict_image_class(img_arr, classes, model_name, model_path, size, transformation):
    print(f'\n - image: size = {img_arr.size}, men={np.mean(img_arr)}, std={np.std(img_arr)}')
    global trained_model
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    # instanciate the model
    clf_model = get_model_instance(model_name, classes)
    # Load the trained model 
    trained_model = load_trained_model(clf_model, model_path)
    # image resizing
    img_arr = resize_image(img_arr, size)
    # apply transforms
    x = transformation(img_arr)  # Preprocess image
    x = x.unsqueeze(0)  # Add batch dimension
    # get predictions
    if device == torch.device('cuda'):
          x = x.cuda()
          # trained_model(data).cpu().argmax(1):
          output = trained_model(x).cpu()  # Forward pass
    else:
          output = trained_model(x)  # Forward pass
    # get pedicted lables/classes
    pred = torch.argmax(output, 1)  # Get predicted class if multi-class classification
    predicted_label = pred[0].cpu().numpy()
    # Normalize scores
    out_scores0 = output.detach().cpu().numpy()[0]
    if np.max(out_scores0)<=0:
      out_scores0 = out_scores0 - np.min(out_scores0)
    out_scores = 100*out_scores0/np.sum(out_scores0)
    # print(f'\n\n Scores={out_scores} \t- label = {predicted_label}')
    pred_score = int(out_scores[predicted_label])
    return predicted_label, pred_score

def predict_image( model_name, model_path, class_file, file_paths, size=(128,128), transformation=None, plotting=False):
    # display 
    print(f'\n\n --> Deploy the following model : \n {model_path} ')
    # load classes
    classes = load_classes(class_file)
    # get the appropriate device
    if isinstance(file_paths, str):
        file_paths=[file_paths]
    
    # run predictions
    pred_classes, pred_scores,file_names = [], [], []
    for idx, file_path in enumerate(file_paths):
        # load the image/slice
        if isinstance(file_path, str):
            img =  load_convert_image(file_path)
            file_name=file_path
        else:
            from PIL import Image
            img =  Image.fromarray(file_path)
            img = img.point(lambda i:i*(1./256)).convert('L')
            file_name=f'slice{idx}'
        # the image prediction
        predicted_label, pred_score = predict_image_class(img_arr=img, classes=classes, model_name=model_name, model_path=model_path, size=size, transformation=transformation)
        pred_classes.append(classes[ str(predicted_label) ] ) # get the class name from dict
        pred_scores.append(pred_score )
        file_names.append(file_name)
        if plotting:
            plot_image(img, 'Predicted class = ' + classes[ str( predicted_label ) ] , filename=file_name)
    # create output pandas
    prediction_df = create_dataframe(file_names, pred_classes, pred_scores)
    return prediction_df

def resize_image(src_image, size): 
    from PIL import Image, ImageOps 
    # resize the image so the longest dimension matches our target size
    src_image.thumbnail(size, Image.ANTIALIAS )
    # Create a new square background image
    new_image = Image.new("RGB", size)
    # Paste the resized image into the center of the square background
    new_image.paste(src_image, (int((size[0] - src_image.size[0]) / 2), int((size[1] - src_image.size[1]) / 2)))
    # return the resized image
    return new_image

def load_convert_image(file_path):
    from PIL import Image
    img =  Image.open(file_path)  # Load image as PIL.Image
    # image conversion to jpg
    if file_path[-4:]==".tif" or file_path[-5:]==".tiff" :
        img = img.point(lambda i:i*(1./256)).convert('L')  
    return img

def plot_image(img, title, filename = ''):
  print(filename)
  # # import Image
  import matplotlib.pyplot as plt
  # for img, filename in zip(imgs, filenames):
  plt.axis('off')
  plt.imshow(img)
  plt.title(title + '\nfile = ' +  filename)
  plt.show()

def create_dataframe(file_paths, pred_classes, pred_score):
  dict = {'file':file_paths,'prediction':pred_classes, 'Confidence-percentage': pred_score}
  return pd.DataFrame(dict)

def prepare_parser():
  parser = ArgumentParser(description='Model deployment')
  parser.add_argument(
      "--model",
      required=True,
      metavar="FILE",
      help="path to the trained model <.pth>",
      type=str,
  )
  parser.add_argument(
      "--model_name",
      required=True,
      help="The trained model architecture",
      type=str,
  )
  parser.add_argument(
      "--class_json",
      default="",
      metavar="FILE",
      help="path to the classes file  <.json>",
      type=str,
  )
  parser.add_argument(
      "--data",
      required=True,
      metavar="DIRECTORY",
      help="Directory where input images are stored."
  )
  parser.add_argument(
      "--resize",
      metavar="image size",
      required=True,
      help="image resizing dimension.",
      type=int,
  )
  parser.add_argument(
      "--dst",
      default="",
      metavar="DIRECTORY",
      help="Directory where results will be stored."
  )
  parser.add_argument(
      "--plot",
      default='1',
      help="show the image with its prediction."
  )

  return parser

def get_input_variable():
    parser = prepare_parser()
    args =  parser.parse_args()
    data_folder=args.data
    model_path = args.model
    model_name=args.model_name
    resize = args.resize
    class_file = args.class_json
    dst_folder = args.dst
    # get the destination folder
    if dst_folder == "":
        dst_folder=os.path.join(os.path.dirname(dst_folder), 'predictions')

    if args.plot=='0':
        plotting=False
    else:
        plotting=True

    return data_folder, model_name, model_path, resize, class_file, dst_folder, plotting

def main_prediction_2D_images():
    transformation = transforms.Compose([
        transforms.ToTensor(),
        # Normalize the pixel values (in R, G, and B channels)
        transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    ])

    # get the input variables
    data_folder, model_name, model_path, resize, class_file, dst_folder, plotting = get_input_variable()

    # list the image to be predicted  
    ext_list = ['.tif', '.tiff', '.jpg']
    list_images_paths = getListOfFiles(data_folder, ext_list) 
    print(f'\n --> Found {len(list_images_paths)} images.\n ')   
    # deploy the classification model 
    prediction_df = predict_image(model_name, model_path, class_file=class_file, file_paths=list_images_paths, \
                                  size=(resize,resize), transformation=transformation, plotting=plotting)
    print(f'\n --> prediction results : \n {prediction_df}')
    # save resut tables
    fileame_csv = os.path.join(dst_folder, 'predictions_ ' + \
                str(os.path.basename(model_path) ) + '.csv')
    create_new_folder(os.path.dirname(fileame_csv))
    prediction_df.to_csv( fileame_csv , sep=',')
    return 0



if __name__ == '__main__':

    main_prediction_2D_images()

    # main_prediction_3D_volumes()
