import time
import threading
import main_data_collection as data_collection
import main_upload_dataset as data_upload



def run():
    # threads for parallel processing
    t1 = threading.Thread(target=data_collection.commpress)
    t2 = threading.Thread(target=data_collection.send)

    t1.start()
    t2.start()

	# threads for parallel processing
    t3 = threading.Thread(target=data_upload.commpress)
    t4 = threading.Thread(target=data_upload.send)

    t1.start()
    t2.start()
    
if __name__ == "__main__":
    run()
