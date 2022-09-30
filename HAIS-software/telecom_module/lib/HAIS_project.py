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
import shutil
import tarfile


Config = {
  "apiKey": "AIzaSyAsPbN83sOHMVcj6O7m7I2PZfse9LLT3J4",
  "authDomain": "testhas-f80bd.firebaseapp.com",
  "projectId": "testhas-f80bd",
  "databaseURL":"gs://testhas-f80bd.appspot.com",
  "storageBucket": "testhas-f80bd.appspot.com",
  "messagingSenderId": "428882133052",
  "appId": "1:428882133052:web:9b4962dc8923ea59448339",
  "measurementId": "G-5NZ08B2LC7",
  "serviceAccount":"serviceAccountKey.json"# abso;lute path here try 
}
root='/home/hais/Desktop/root'
row_data_path=r"/home/hais/Desktop/sweeps"# chage here os.getcwd()
download=r"/home/hais/Desktop/DOWNLOAD_DATABASE"#chenge here
firebase_storage= pyrebase.initialize_app(Config)
storage=firebase_storage.storage()
database = firebase_storage.database()

sensor_files=os.listdir(row_data_path)  


def create_new_folder(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)
    
create_new_folder(root);create_new_folder(download)
    
try:  
    for i in sensor_files:
        os.mkdir(os.path.join(download,i))
except:
    pass
#database = firebase_storage.database()
def chek_the_data():
    fil=0
    for roo, dirs, files in os.walk(row_data_path, topdown=False):
        fil+=len(files)
    #this function return True if we have new data,False if not 
    if fil!=0  :# or ['test.jason'] to make a file constant
        return True
    else:
        return False
os.path.getsize(row_data_path)
def remove_from_root(file_name):
    #this function remove the file from the root folder 
    os.rmdir(root,file_name)
def remove_from_row(file_name):
    #this function remove the file from the root folder 
    os.remove(os.path.join(row_data_path,file_name))

def getitout():
    for roo, dirs, files in os.walk(row_data_path):
        for name in dirs:
            file=os.listdir(os.path.join(roo, name))
            for i in file:
                os.rename(os.path.join(roo,name, i),os.path.join(row_data_path, i))
            os.removedirs(os.path.join(roo, name))
    return (dirs)
        
def commpresss(th=1000000):
    
    os.chdir(row_data_path)
    new=str(time.ctime()).replace(' ','_').replace(':','_')
    filename= f"{new}.tar"
    file_obj= tarfile.open(filename,"w")
    size=0
    c=0
    for roo, dirs, files in os.walk(row_data_path, topdown=False):
        for fi in files:
            try:
                shutil.move(os.path.join(roo,fi), os.path.join(row_data_path,fi))
            except:
                pass
    for fi in os.listdir(row_data_path):
                if '.tar' not in fi and fi not in sensor_files: 
                    data= fi
                    size+=os.path.getsize(data)
                    if size <= th:
                        file_obj.add(data)
                        os.remove(data)
                    else:
                        c+=1
                        file_obj.close()
                        shutil.move(os.path.join(row_data_path,filename), os.path.join(root,filename))
                        new=str(time.ctime()).replace(' ','_').replace(':','_')
                        filename= f"{new}{size}{c}.tar"
                        file_obj= tarfile.open(filename,"w")
                        file_obj.add(data)
                        size=os.path.getsize(data)
                        os.remove(data)
    file_obj.close()
    shutil.move(os.path.join(row_data_path,filename), os.path.join(root,filename))
     
def send_data():
          lis=os.listdir(root)
          print(f' \n - Sending {len(lis)} files to the firebase. Please wait... \n')
          for name in lis:
              bath = os.path.join(root,name)
              try:
                  storage.child(name).put(bath)
                  os.remove(bath)
              except:
                  print('connection error...')

def get_data():
    all_files=storage.list_files()
    cnt=0
        
        
    
    print(f'\n- Getting data from the cloud ...')
    for file in all_files:# we can add any kind of condition that will help the idea of the project
        cnt+=1
        if cnt%10:
            print(f'.')
        file.download_to_filename(os.path.join(download,file.name))
        file.delete()#delete the file 
   
    if cnt==0:
        print(f'\n- No data are available on cloud!!')
        return True
    else:
        return False
    
def decommpress():  
    com = [f for f in os.listdir(download) if os.path.isfile(os.path.join(download, f))]

    for c in range(len(com)):
        my_tar = tarfile.open(os.path.join(download,com[c]))
        my_tar.extractall(download) # specify which folder to extract to
        my_tar.close()
#        os.remove(os.path.join(download,com[c]))
    tar = [f for f in os.listdir(download) if os.path.isfile(os.path.join(download, f)) and '.tar' in f]
    for c in tar:
        os.remove(os.path.join(download,c))
    for file in sensor_files:
        direction=os.path.join(download,file)
        com = [f for f in os.listdir(download) if os.path.isfile(os.path.join(download, f))]
        for name in com:
            if file.lower() in name.lower():
                os.replace(os.path.join(download,name),os.path.join(direction,name))

def delet_firebase():
    all_files=storage.list_files()
    for file in all_files:# we can add any kind of condition that will help the idea of the project
        file.delete()
            
        
        
        
        
