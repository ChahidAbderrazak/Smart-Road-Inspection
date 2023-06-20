import os, json, shutil, time
from typing import List
from glob import glob
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, File, UploadFile, Request, Form
from fastapi.responses import HTMLResponse

from PIL import Image
from io import BytesIO
import numpy as np 
from lib.Autils_classification import predict_image
from lib import utils_hais


# download_path=os.path.join(os.getcwd(),'download')
download_path='/media/abdo2020/DATA1/data/labeled-dataset/HAIS-project/download'
max_dist=10   # maximal distance for the data to be displayed
full_inspection_dict_path= os.path.join(os.path.dirname(download_path),'database', 'inspection_dic.json')
# picked_location={"lat":43.937092, "lon":-78.867443}
# dict_sensor=utils_hais.search_node_in_DB(download_path,	picked_location, disp=True)
# print(f'\n sensor dict={dict_sensor}')

app = FastAPI()

# download_path='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31'
try:
    app.mount("/static", StaticFiles(directory="static"), name="static")
    templates = Jinja2Templates(directory="templates/")
except:
    app.mount("/static", StaticFiles(directory="src/static"), name="static")
    templates = Jinja2Templates(directory="src/templates/")    



@app.get("/")
def form_post(request: Request):
    return templates.TemplateResponse('map.html', context={'request': request})

@app.get("/ai")
def form_post(request: Request):
    image_path = "static/files/prediction.jpg"
    status_AI="Uplaoding an image"
    predictions=0
    # prediction using dumy example
    import random 
    scores=[10,20,37,80,50,65]
    random.shuffle(scores)
    probabilities = np.array(scores)
    classes = ['small-hole', 'big-hole', 'small-crack', 'big-crack', 'good-road', 'undefined']
    index = np.argsort(probabilities)
    predictions = {
      "class1":classes[index[-1]],
      "class2":classes[index[-2]],
      "class3":classes[index[-3]],
      "prob1":probabilities[index[-1]],
      "prob2":probabilities[index[-2]],
      "prob3":probabilities[index[-3]],
    }
    return templates.TemplateResponse('ai-diagnosis.html', context={'request': request, 'image_path': image_path, 'predictions': predictions,'status_AI': status_AI})

def read_imagefile(data) -> Image.Image:
    image = Image.open(BytesIO(data))
    return image

@app.post("/ai")
async def upload_file(request: Request, file: UploadFile = File(...)):
    # Step 0: Get the image from the user 
    image = read_imagefile(await file.read())
    image_path="static/files/processed_"+file.filename
    image = image.save(image_path)

    # Step 1: Load the image 
    print(f'\n image_path={image_path}')
    my_image=Image.open(image_path)

    #Step 2 : Resize the image if needed
    my_image_re = my_image.resize((128,128))
    print('Uploaded image (resized):',my_image_re)
    
    # #Step 3 : prediction
    # import random 
    # scores=[10,20,37,80,50,65]
    # random.shuffle(scores)
    # probabilities = np.array(scores)
    # #Step 4
    # classes = ['defect-free', 'missing-screw', 'displaced-screw', 'scratch', 'bending', 'other-defect']
    # index = np.argsort(probabilities)
    # predictions = {
    #   "class1":classes[index[-1]],
    #   "prob1":probabilities[index[-1]],
    # }

    # status_AI=" ( Dumy prediction )"

    # Step 3 :prediction using model:
    # deploy the classification model 
    classes = ['road holes', 'road cracks']
    model_path = 'model/model_cpu.pth'
    output = predict_image(model_path, classes, [image_path], plotting=False)
    pred_label, score = output['prediction'].values[0], output['score'].values[0]
    print(f'\n\n model predictions results = \n {output}')
    status_AI=" ( Prediction = " + pred_label +")"

    predictions = {
      "class1": pred_label,
      "prob1": score,
    }
    return templates.TemplateResponse('ai-diagnosis.html', context={'request': request, 'image_path': "../"+image_path, 'predictions': predictions, 'status_AI': status_AI})

@app.get("/reports")
def form_post(request: Request):
    image_path = "  "
    
    return templates.TemplateResponse('reports.html', context={'request': request, 'image_path': image_path})

@app.get("/sensors")
def form_post(request: Request):
    image_path = "  "
    return templates.TemplateResponse('sensors.html', context={'request': request, 'image_path': image_path})

@app.get("/via-annotator")
def form_post(request: Request):
    data_root = "download/"
    return templates.TemplateResponse('via-annotator.html', context={'request': request, 'data_root': data_root})


@app.get("/contact")
def form_post(request: Request):
    image_path = "  "
    return templates.TemplateResponse('contact.html', context={'request': request, 'image_path': image_path})


@app.get("/get_node_list")
async def get_sensor_data():
    list_nodes=[ os.path.basename(path) for path in glob(os.path.join(download_path,'*')) if os.path.isdir(path)] 
    dict_list_nodes={node:idx+1 for idx, node in enumerate(list_nodes)}
    data = serialize_sensor_dict(dict_list_nodes)
    # print(f'\n List of found nodes data={data}')
    return data

@app.get("/get_routes/{node_name}")
async def read_user_me(node_name):
    print(f'\n requestd node is : {node_name}')
    f=open(os.path.join(download_path,node_name,'inspection_dic.json'))
    data=json.load(f)
    classes= np.unique(data['metric'])
    # print(f'\n - requestd node is : {node_name} \
    #         \n - inspected road classes= {classes} \
    #         \n data ={data}')
    return data


@app.get("/get_sensor_data/{location_str}")
async def get_sensor_data(location_str):
    #Clear the temporary folder of the sensor data
    tmp_dir='static/tmp'
    utils_hais.clean_directory(tmp_dir)

    # get the closly collected data tothe picked location
    location=location_str.split('__')
    picked_location={"lat":float(location[0]), "lon":float(location[1])}
    print(f'\n the picked location data is : {picked_location}')
    dict_sensor=utils_hais.search_node_in_DB(database_root=download_path, picked_location=picked_location, max_dist=max_dist)
    # copy the files locally
    copy_sensor_data(dict_sensor, tmp_dir)
    # flag: serialize the json [quick solution as the <dict_sensor> does not POST correctly ]
    data = serialize_sensor_dict(dict_sensor)
    print(f'\n sensor data={data}')
    return data


def copy_sensor_data(dict_sensor, tmp_dir):
    
    try:
        dst_cam= os.path.join(tmp_dir, os.path.basename(dict_sensor['camera']) )
        shutil.copy(dict_sensor['camera'], dst_cam) 
        shutil.copy(dict_sensor['camera'], os.path.join(tmp_dir,'cam.jpg') )
    except Exception as e: 
        dst_cam=''
        print(f'\n Error in loading the camera data!! \n Exception: {e}')
    dict_sensor['camera']=dst_cam
    # lidar
    
    try: 
        dst_lidar= os.path.join(tmp_dir, os.path.basename(dict_sensor['lidar']) ) 
        shutil.copy(dict_sensor['lidar'], dst_lidar) 
        shutil.copy(dict_sensor['lidar'], os.path.join(tmp_dir,"lidar.gif")) 
    except Exception as e:
        dst_lidar='' 
        print(f'\n Error in loading the lidar data!! \n Exception: {e}')
    dict_sensor['lidar']=dst_lidar
    
def serialize_sensor_dict(dict_sensor):
    tmp_filename='temp/dict_sensor.json'
    utils_hais.save_json(dict_sensor, tmp_filename)
    f=open(tmp_filename)
    data=json.load(f)
    
    return data