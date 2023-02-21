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

app = FastAPI()

# download_path='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31'

app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates/")


@app.get("/")
def form_post(request: Request):
    return templates.TemplateResponse('index.html', context={'request': request})
