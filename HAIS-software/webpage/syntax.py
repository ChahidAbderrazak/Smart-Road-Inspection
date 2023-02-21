import os, json
from typing import List
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, Response, File, UploadFile, Request, Form
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.encoders import jsonable_encoder
from pydantic import BaseModel
from typing import Union

from PIL import Image
from io import BytesIO
import numpy as np 
from lib.Autils_classification import predict_image
from datetime import datetime
from typing import Union


dict_sensor={'camera': "../static/tmp/cam.jpg", 
            'lidar': "../static/tmp/lidar.gif", 
            'Kenematics ': "ACCX Angle=45.00000038242425", 
            'weather':'ACCX Angle=45.00000038242425'}


app = FastAPI()
download_path=os.path.join(os.getcwd(),'download')
# download_path='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31'
try:
    app.mount("/static", StaticFiles(directory="static"), name="static")
    templates = Jinja2Templates(directory="templates/")
except:
    app.mount("/static", StaticFiles(directory="src/static"), name="static")
    templates = Jinja2Templates(directory="src/templates/")    

@app.get('/')
def read_form(request: Request): 
    result ='home'
    return templates.TemplateResponse('homepage.html', context={'request': request, 'result': result}) 

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

@app.get("/map")
def form_post(request: Request):
    return templates.TemplateResponse('map.html', context={'request': request})

# @app.get("/routes/{node_name}", response_class=HTMLResponse)
# def form_post(request: Request, node_name):
#     # node_name='node1'
#     f=open(os.path.join(download_path,node_name,'inspection_dic.json'))
#     data=json.load(f)
#     return templates.TemplateResponse('map.html', context={'request': request, 'node_name':node_name,  'json_routes': data})

@app.get("/contact")
def form_post(request: Request):
    image_path = "  "
    return templates.TemplateResponse('contact.html', context={'request': request, 'image_path': image_path})


### Ahamd API 
@app.route('/get_routes/<string:node_name>/')
def get_routes(node_name):
    print(f'\n requestd node is : {node_name}')
    f=open(os.path.join(download_path,node_name,'inspection_dic.json'))
    data=json.load(f)
    return data

@app.route('/get_routes2/<string:node_name>/')
def get_routes2(node_name):
    data=json.dumps(dict_sensor)
    return data

@app.route('/get_popup_data/')
def  get_picked_location(request: Request):
    print(f'\n picked location=({request})')
    # print(f'\n picked location=({lat},{lng})')
    # data={'CSI_CAMERA': "../static/tmp/cam.jpg", 
    #     'IMU_SENSOR':{'ACCX Angle': 45.00000038242425, 
    #                 'ACCY Angle': -44.0, 
    #                 'GRYX Angle': 0.0133, 
    #                 'GYRY Angle': -0.360, 
    #                 'GYRZ Angle': -0.30, }
    # }       

    data={"d":"djhjgj"}       
    return json.dumps(data)



class sensor_data(BaseModel):
    camera: str
    lidar: str
    Kenematics: str
    weather: str


@app.get("/legacy/<string:picked_location>/")
def get_legacy_data(picked_location):
    print(f'\n picked location=({picked_location})')
    return Response(content=json.dumps(dict_sensor) )
    
# @app.get("/get_routes/{node_name}")
# async def read_user_me(node_name):
#     print(f'\n requestd node is : {node_name}')
#     f=open(os.path.join(download_path,node_name,'inspection_dic.json'))
#     data=json.load(f)
#     classes= np.unique(data['metric'])
#     print(f'\n - requestd node is : {node_name} \n - inspected road classes= {classes}')
#     return data

