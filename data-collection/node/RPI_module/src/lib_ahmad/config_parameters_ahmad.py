import os
import pyrebase
def create_new_folder(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)

###### intialize paramaters
data_root = "../upload"  															# storage path on local device
download='../download'  															# storage path on firebase download
root='../tmp'
# Firebase config
Config = {
  "apiKey": "AIzaSyAsPbN83sOHMVcj6O7m7I2PZfse9LLT3J4", 
  "authDomain": "testhas-f80bd.firebaseapp.com", 
  "projectId": "testhas-f80bd", 
  "databaseURL":"gs://testhas-f80bd.appspot.com", 
  "storageBucket": "testhas-f80bd.appspot.com", 
  "messagingSenderId": "428882133052", 
  "appId": "1:428882133052:web:9b4962dc8923ea59448339", 
  "measurementId": "G-5NZ08B2LC7", 
  "serviceAccount":"lib_ahmad/serviceAccountKey.json"# abso;lute path here try 
}
row_data_path=data_root
firebase_storage= pyrebase.initialize_app(Config)
storage=firebase_storage.storage()
database = firebase_storage.database()

# create the folders
create_new_folder(data_root)
create_new_folder(download)
create_new_folder(root)