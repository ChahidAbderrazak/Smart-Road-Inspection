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
import json
from lib_ahmad.config_parameters_ahmad import *

# Config = {
#   "apiKey": "AIzaSyAsPbN83sOHMVcj6O7m7I2PZfse9LLT3J4", 
#   "authDomain": "testhas-f80bd.firebaseapp.com", 
#   "projectId": "testhas-f80bd", 
#   "databaseURL":"gs://testhas-f80bd.appspot.com", 
#   "storageBucket": "testhas-f80bd.appspot.com", 
#   "messagingSenderId": "428882133052", 
#   "appId": "1:428882133052:web:9b4962dc8923ea59448339", 
#   "measurementId": "G-5NZ08B2LC7", 
#   "serviceAccount":"serviceAccountKey.json"# abso;lute path here try 
# }
# try:
#     os.mkdir('root')
# except:
#     pass
# try:
#     os.mkdir('download')
# except:
#     pass
# root=os.path.join(os.getcwd(), 'root')
# row_data_path=os.path.join(os.getcwd(), 'upload')# chage here os.getcwd()

# download=os.path.join(os.getcwd(), 'download')
# firebase_storage= pyrebase.initialize_app(Config)
# storage=firebase_storage.storage()
# database = firebase_storage.database()

def chek_the_data():
    fil=0
    for roo, dirs, files in os.walk(row_data_path, topdown=False):
        fil+=len(files)
    #this function return True if we have new data, False if not 
    if fil!=0  :# or ['test.jason'] to make a file constant
        return True
    else:
        return False

def remove_from_root(file_name):
    #this function remove the file from the root folder 
    os.rmdir(root, file_name)

def remove_from_row(file_name):
    #this function remove the file from the root folder 
    os.remove(os.path.join(row_data_path, file_name))

def getitout():
    for roo, dirs, files in os.walk(row_data_path):
        for name in dirs:
            file=os.listdir(os.path.join(roo, name))
            for i in file:
                os.rename(os.path.join(roo, name, i), os.path.join(row_data_path, i))
            os.removedirs(os.path.join(roo, name))
    return (dirs)
        
def commpresss(th=1000000):
    origen=os.getcwd()
    nodes=os.listdir(row_data_path)
    for n in nodes:
        node_path=os.path.join(row_data_path, n)
        try:
            os.chdir(node_path)
        except Exception as e:
            print(f'\n dataset localization error! \n Exception: {e}')
            continue 
        row_data_sweep=os.path.join(node_path, 'sweeps')
        x=open('sweeps_stur.txt', 'x')
        for roo, dirs, files in os.walk(row_data_sweep):
            for di in files:
                x.write(str(roo.split('\\')[-1])+'\t @'+str(di)+'\n')
        x.close()
        # dataset info
        try:
            info=open(os.path.join(node_path, 'info.json'))
            info=json.load(info)['vehicle']
            node_name=info+'___'
            fil = [f for f in os.listdir(node_path) if os.path.isfile(os.path.join(node_path, f))]
        except Exception as e:
            print(f'\n dataset definition error! \n Exception: {e}')
            continue 
        # compress the different olders: maps, missions, etc
        for folder in ['map', 'mission', 'v1.0', 'meta']:
            try:
                mapname=node_name+folder+"_"+str(time.ctime()).replace(' ', '_').replace(':', '_')+'.tar'
                file_obj= tarfile.open(mapname, "w")
                file_obj.add(folder)
                shutil.rmtree(folder)
                file_obj.close()
            except Exception as e:
                print(f'\n Compression error! \n Exception: {e}')

        # mapname=node_name+"mission_"+str(time.ctime()).replace(' ', '_').replace(':', '_')+'.tar'
        # file_obj= tarfile.open(mapname, "w")
        # file_obj.add('missions')
        # shutil.rmtree('missions')
        # file_obj.close()
        # mapname=node_name+"v1_"+str(time.ctime()).replace(' ', '_').replace(':', '_')+'.tar'
        # file_obj= tarfile.open(mapname, "w")
        # file_obj.add('v1.0')
        # shutil.rmtree('v1.0')
        # file_obj.close()

        # meta file
        mapname=node_name +"meta_"+str(time.ctime()).replace(' ', '_').replace(':', '_')+'.tar'
        file_obj= tarfile.open(mapname, "w")
        for i in fil:
            file_obj.add(i)
            os.remove(i)
        file_obj.close()

        # copy files
        for roo, dirs, files in os.walk(row_data_sweep, topdown=False):
            for fi in files:
                try:
                    shutil.move(os.path.join(roo, fi), fi)
                except:
                    pass
        mapname=node_name+"sweeps_"+str(time.ctime()).replace(' ', '_').replace(':', '_')+'.tar'
        file_obj= tarfile.open(mapname, "w")
        file_obj.add('sweeps')
        shutil.rmtree('sweeps')
        file_obj.close()
        new=str(time.ctime()).replace(' ', '_').replace(':', '_')
        filename=node_name+ f"{new}.tar"
        file_obj= tarfile.open(filename, "w")
        size=0
        c=0
        dat=[f for f in os.listdir(node_path) if  '.tar' not in f]
        for fi in dat:
            data= fi
            size+=os.path.getsize(data)
            if size <= th:
                file_obj.add(data)
                os.remove(data)
            else:
                c+=1
                file_obj.close()
                new=str(time.ctime()).replace(' ', '_').replace(':', '_')
                filename=node_name+ f"{new}{size}{c}.tar"
                file_obj= tarfile.open(filename, "w")
                file_obj.add(data)
                size=os.path.getsize(data)
                os.remove(data)
        file_obj.close()
        for i in os.listdir(node_path):
            shutil.move(os.path.join(node_path, i), os.path.join(root, i))
        os.chdir(origen)
     
def send_data():
          lis=os.listdir(root)
          for name in lis:
              bath = os.path.join(root, name)
              node=name.split('___')[0]
              try:
                  storage.child(f'{node}/{name}').put(bath)
                  os.remove(bath)
              except:
                  print('connection error...')

def get_data():
    all_files=storage.list_files()
    for file in all_files:# we can add any kind of condition that will help the idea of the project
        node_name=file.name.split('/')[0]
        fi=file.name.split('/')[-1]
        try:
            path=os.path.join(download, node_name)
            os.mkdir(path)
        except:
            pass
# download the sturcher 


def decommpress(node):
    get_data()
    all_files=storage.list_files()
    for file in all_files:# we can add any kind of condition that will help the idea of the project
        node_name=file.name.split('/')[0]
        if node_name.lower() == node.lower():
            fi=file.name.split('/')[-1]
            path=os.path.join(download, node_name)
            file.download_to_filename(os.path.join(path, fi))
            #file.delete()#delete the file 
    path=os.path.join(download, node)
    com = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    for c in range(len(com)):
        my_tar = tarfile.open(os.path.join(path, com[c]))
        my_tar.extractall(path) # specify which folder to extract to
        my_tar.close()
        os.remove(os.path.join(path, com[c]))
    f=open(os.path.join(path, 'sweeps_stur.txt'), 'r')
    f=f.readlines()
    for i in f:
            i=i.split('	 @')
            folder=i[0]
            data=i[1][:-1]
            shutil.move(os.path.join(path, data), os.path.join(path, 'sweeps', folder, data))


def delet_firebase(): 
    all_files=storage.list_files()
    for file in all_files:# we can add any kind of condition that will help the idea of the project
        file.delete()
            
        
        
       
