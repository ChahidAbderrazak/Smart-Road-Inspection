import cv2, sys, os, shutil
from pathlib import Path

video_name = sys.argv[1]
dot_index = video_name.index('Vid1')
video_name = video_name[0 : dot_index]

IMAGE_DIR = f'Images/{ video_name }'

if not os.path.exists(IMAGE_DIR):
    os.makedirs(IMAGE_DIR)

vidcap = cv2.VideoCapture("Vid1.mp4")
def getFrame(sec):
    vidcap.set(cv2.CAP_PROP_POS_MSEC,sec)
    hasFrames,image = vidcap.read()
    if hasFrames:
        cv2.imwrite(f'image{ str(count) }.jpg', image) # save frame as JPG file
        shutil.move(f'image{ str(count) }.jpg', IMAGE_DIR)
    return hasFrames
sec = 1
frameRate = 30 #//it will capture image in each 0.5 second
count=1
success = getFrame(sec)
while success:
    
    count = count + 1
    sec = sec + frameRate
    sec = round(sec, 2)
    success = getFrame(sec)
    print(f' saving frame{count} --> success={success}')
