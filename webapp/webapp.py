import json
import os
import shutil
from glob import glob
from io import BytesIO

import numpy as np
import uvicorn
from fastapi import FastAPI, File, Request, UploadFile
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from PIL import Image

from lib import utils_hais
from lib.Autils_classification import predict_image

IMAGE_DEMO = "static/files/prediction.jpg"
IMAGE_ERROR = "static/files/error.jpg"

CONFIG_FILE = "config/config.yml"
# ----------------------------------------------
max_dist = 10   # maximal distance for the data to be displayed
# download_path = utils_hais.get_data_folder()
download_path = 'data/download'
print(f'\n - The data folder is: {download_path}')
# download_path=os.path.join(os.path.dirname(os.getcwd()),'data','download')
# if not os.path.exists(download_path):
#     # get the parent root folder
#     download_path=os.path.join(os.path.dirname(os.path.dirname(os.getcwd())),'data','download')

if not os.path.exists(download_path):
    msg = f'Error: The data folder is not found: {download_path}\n Please download the sample dataset as explained in the README.md'
    raise Exception(msg)
# ----------------------------------------------
app = FastAPI()

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
    image_path = IMAGE_DEMO
    status_AI = "Uploading an image"
    predictions = 0
    # prediction using dumy example
    import random
    scores = [10, 20, 37, 80, 50, 65]
    random.shuffle(scores)
    probabilities = np.array(scores)
    classes = ['small-hole', 'big-hole', 'small-crack',
               'big-crack', 'good-road', 'undefined']
    index = np.argsort(probabilities)
    predictions = {
        "class1": classes[index[-1]],
        "class2": classes[index[-2]],
        "class3": classes[index[-3]],
        "prob1": probabilities[index[-1]],
        "prob2": probabilities[index[-2]],
        "prob3": probabilities[index[-3]],
    }
    return templates.TemplateResponse('ai-diagnosis.html', context={'request': request, 'image_path': image_path, 'predictions': predictions, 'status_AI': status_AI})


def read_imagefile(data) -> Image.Image:
    image = Image.open(BytesIO(data))
    return image


@app.post("/ai")
async def upload_file(request: Request, file: UploadFile = File(...)):
    # Step 0: Get the image from the user
    image = read_imagefile(await file.read())
    image_path = "static/files/processed_"+file.filename
    image = image.save(image_path)

    # Step 1: Load the image
    print(f'\n image_path={image_path}')
    my_image = Image.open(image_path)

    # Step 2 : Resize the image if needed
    my_image_re = my_image.resize((128, 128))
    print('Uploaded image (resized):', my_image_re)

    # Step 3 :prediction using model:
    # deploy the classification model
    classes = ['road holes', 'road cracks']
    model_path = 'models/model_cpu.pth'
    try:
        # //TODO: inference of the trained model
        if os.path.exists(model_path):
            output = predict_image(model_path, classes, [
                image_path], plotting=False)
            print(f'\n - output={output}')
            pred_label = output['prediction'].values,
            score = output['score'].values
            print(f'\n\n model predictions results = \n {output}')
            status_AI = " ( Prediction = " + pred_label + ")"

            predictions = {
                "class1": pred_label,
                "prob1": score,
            }

        else:
            image_path = IMAGE_ERROR
            status_AI = f"Error: (cannot find the trained model path : [{model_path}])"
            predictions = {
                "class1": "NA",
                "prob1": 0.0,
            }
    except Exception as e:
        image_path = IMAGE_ERROR
        status_AI = f"Error: \n {e}"
        predictions = {
            "class1": "NA",
            "prob1": 0.0,
        }

    return templates.TemplateResponse('ai-diagnosis.html',
                                      context={'request': request,
                                               'image_path': "../"+image_path,
                                               'predictions': predictions,
                                               'status_AI': status_AI})


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
    list_nodes = [os.path.basename(path) for path in glob(
        os.path.join(download_path, '*')) if os.path.isdir(path)]
    dict_list_nodes = {node: idx+1 for idx, node in enumerate(list_nodes)}
    data = serialize_sensor_dict(dict_list_nodes)
    print(f'\n Nodes data directory={download_path}')
    print(f'\n List of found nodes data={data}')
    return data


@app.get("/get_routes/{node_name}")
async def read_user_me(node_name):
    print(f'\n requested node is : {node_name}')
    f = open(os.path.join(download_path, node_name, 'inspection_dic.json'))
    data = json.load(f)
    classes = np.unique(data['metric'])
    # print(f'\n - requested node is : {node_name} \
    #         \n - inspected road classes= {classes} \
    #         \n data ={data}')
    return data


@app.get("/get_sensor_data/{location_str}")
async def get_sensor_data(location_str):
    # Clear the temporary folder of the sensor data
    tmp_dir = 'static/tmp'
    utils_hais.clean_directory(tmp_dir)

    # get the closely collected data to the picked location
    location = location_str.split('__')
    picked_location = {"lat": float(location[0]), "lon": float(location[1])}
    print(f'\n the picked location data is : {picked_location}')
    dict_sensor = utils_hais.search_node_in_DB(
        database_root=download_path, picked_location=picked_location, max_dist=max_dist)
    # copy the files locally
    copy_sensor_data(dict_sensor, tmp_dir)
    # TODO flag: serialize the json [quick solution as the <dict_sensor> does not POST correctly ]
    data = serialize_sensor_dict(dict_sensor)
    print(f'\n sensor data={data}')
    return data


def copy_sensor_data(dict_sensor, tmp_dir):

    try:
        dst_cam = os.path.join(
            tmp_dir, os.path.basename(dict_sensor['camera']))
        shutil.copy(dict_sensor['camera'], dst_cam)
        shutil.copy(dict_sensor['camera'], os.path.join(tmp_dir, 'cam.jpg'))
    except Exception as e:
        dst_cam = ''
        print(f'\n Error in loading the camera data!! \n Exception: {e}')
    dict_sensor['camera'] = dst_cam
    # lidar

    try:
        dst_lidar = os.path.join(
            tmp_dir, os.path.basename(dict_sensor['lidar']))
        shutil.copy(dict_sensor['lidar'], dst_lidar)
        shutil.copy(dict_sensor['lidar'], os.path.join(tmp_dir, "lidar.gif"))
    except Exception as e:
        dst_lidar = ''
        print(f'\n Error in loading the lidar data!! \n Exception: {e}')
    dict_sensor['lidar'] = dst_lidar


def serialize_sensor_dict(dict_sensor):
    tmp_filename = 'static/tmp/dict_sensor.json'
    utils_hais.save_json(dict_sensor, tmp_filename)
    f = open(tmp_filename)
    data = json.load(f)

    return data


def prepare_parser():
    from argparse import ArgumentParser
    parser = ArgumentParser(description='Code template')
    parser.add_argument(
        "--port",
        default=8080,
        metavar="input port",
        help="API server port",
        type=str,
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        metavar="host IP address",
        help="API server IP address",
        type=str,
    )

    return parser


if __name__ == '__main__':
    # get the input parameters
    parser = prepare_parser()
    args = parser.parse_args()

    # run the UVICORN server
    uvicorn.run(app, host=args.host, port=args.port)
