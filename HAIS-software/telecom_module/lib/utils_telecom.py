# -*- coding: utf-8 -*-
"""
@author: Ahmad Mousa 100840150
"""

"""in this code we will send the data to fire base 
make sure you run this line in the first time run:
pip install Pyrebase
"""
import pyrebase
import time
import os 


# ## Ahmad Moussa
# Config = {
#   "apiKey": "AIzaSyAsPbN83sOHMVcj6O7m7I2PZfse9LLT3J4",
#   "authDomain": "testhas-f80bd.firebaseapp.com",
#   "projectId": "testhas-f80bd",
#   "databaseURL":"gs://testhas-f80bd.appspot.com",
#   "storageBucket": "testhas-f80bd.appspot.com",
#   "messagingSenderId": "428882133052",
#   "appId": "1:428882133052:web:9b4962dc8923ea59448339",
#   "measurementId": "G-5NZ08B2LC7",
#   "serviceAccount":"/content/drive/MyDrive/HAIS project /serviceAccountKey.json"

# }
# root=r'C:\Users\100840150\Desktop\HAIS_DEMO\root'
# row_data_path=r'C:\Users\100840150\Desktop\HAIS_DEMO\datareceiver'
# download=r'C:\Users\100840150\Desktop\HAIS_DEMO\download'


##abderraazak
# firebase storage configuration
Config = {'apiKey': "AIzaSyCBTp3caunMJ6JlXyNXlN0DERVLi8EJ6Ho",
        'authDomain': "hais-project-d9692.firebaseapp.com",
        'databaseURL': "https://hais-project-d9692-default-rtdb.firebaseio.com",
        'projectId': "hais-project-d9692",
        'storageBucket': "hais-project-d9692.appspot.com",
        'messagingSenderId': "926868745531",
        'appId': "1:926868745531:web:6073d481701edc27c51b1e",
        'measurementId': "G-25QKT2MGB6",
        "serviceAccount":"serviceAccountKey.json"
        
    }

root=r'data/root'
row_data_path=r'data/datareceiver'
download=r'data/download'

# firebase
firebase= pyrebase.initialize_app(Config)
storage=firebase.storage()

def chek_the_data():
    #this function return True if we have new data,False if not 
    return os.listdir(row_data_path)!=[]

def remove_from_root(file_name):
    #this function remove the file from the root folder 
    os.rmdir(root,file_name)

def rename():
    lis=os.listdir(row_data_path)
    for name in lis:
        print(f'\n - name= {name} \n - {str(time.ctime())+name}')
        print(f'\n - row_data_path= {row_data_path} \n - root={root}')
        src_filename=os.path.join(row_data_path,name)
        dst_filename=os.path.join(root,str(time.ctime())+name)
        os.rename(src_filename, dst_filename)
        
def send_data():
    lis=os.listdir(root)
    print(f'\n\n - Syncronizing {len(lis)} files:')
    sent_data=[]
    for name in lis:
        path = os.path.join(root,name)
        try:
            storage.child(name).put(path)
            # os.remove(os.path.join(root,name))
            sent_data.append(path)
        except:
            print('connection error...')
    print(f'\n\n    -> Diagnosis: {len(sent_data)} sent {len(lis) - len(sent_data)} faild/')

def download_data():
    all_files=storage.list_files()
    for file in all_files:# we can add any kind of condition that will help the idea of the project
        file.download_to_filename(os.path.join(download,file.name))

def upload_data():
    while True:
        if chek_the_data():
            rename()
            send_data()
            time.sleep(0.5)
        else:
            print('there is no data yet')
            time.sleep(0.5)