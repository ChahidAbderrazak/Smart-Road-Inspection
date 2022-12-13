from genericpath import isfile
import os

DATA_DIR = '/run/user/1003/gvfs/smb-share:server=sesl-cloud.local,share=hais-project/dataset/dash-cam/annotated-videos/test2/TEST_ROUTE-3' 
d= os.walk(DATA_DIR) #os.path.isfile(DATA_DIR)
# videos_list=[vid for vid in os.walk(DATA_DIR) if os.path.isfile(vid) ]
# d= next(videos_list)
print(d)